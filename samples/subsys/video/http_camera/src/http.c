/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/video/video.h>

#include "app.h"

LOG_MODULE_REGISTER(http, LOG_LEVEL_INF);

#define HTTP_PORT 80

/* One more worker than streams, so that the page and snapshots are always served */
#define WORKERS      (MAX_STREAMS + 1)
#define STACK_SIZE   4096
#define REQUEST_SIZE 512

#define BOUNDARY "frame"

static const uint8_t index_html_gz[] = {
#include "index.html.gz.inc"
};

static atomic_t sent;
static atomic_t streams;

K_MSGQ_DEFINE(client_q, sizeof(int), WORKERS, 4);
static K_THREAD_STACK_ARRAY_DEFINE(worker_stacks, WORKERS, STACK_SIZE);
static struct k_thread workers[WORKERS];
static K_THREAD_STACK_DEFINE(accept_stack, 2048);
static struct k_thread accept_thread;

static int send_all(int fd, const void *buf, size_t len)
{
	const uint8_t *p = buf;
	ssize_t ret;

	while (len > 0) {
		ret = zsock_send(fd, p, len, 0);
		if (ret < 0) {
			return -errno;
		}

		atomic_add(&sent, ret);
		p += ret;
		len -= ret;
	}

	return 0;
}

static int send_str(int fd, const char *s)
{
	return send_all(fd, s, strlen(s));
}

static void send_status(int fd, const char *status)
{
	char head[128];

	snprintf(head, sizeof(head),
		 "HTTP/1.1 %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n", status);
	send_str(fd, head);
}

static void serve_index(int fd)
{
	char head[160];

	snprintf(head, sizeof(head),
		 "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Encoding: gzip\r\n"
		 "Content-Length: %u\r\nConnection: close\r\n\r\n",
		 sizeof(index_html_gz));
	if (send_str(fd, head) == 0) {
		send_all(fd, index_html_gz, sizeof(index_html_gz));
	}
}

static void serve_stats(int fd)
{
	struct camera_stats st;
	char body[448];
	char head[128];
	uint32_t channel;
	uint32_t phy;
	int rssi;
	int len;

	camera_get_stats(&st);
	wifi_get_status(&rssi, &phy, &channel);

	len = snprintf(body, sizeof(body),
		       "{\"uptime\":%u,\"width\":%u,\"height\":%u,\"qs\":%u,"
		       "\"fps\":%u.%u,\"interval_us\":%u,\"jitter_us\":%u,"
		       "\"frame_avg\":%u,\"frame_max\":%u,\"frames\":%u,\"bytes\":%u,"
		       "\"dropped\":%u,\"snapshots\":%u,\"snap_size\":%u,\"snap_pause_ms\":%u,"
		       "\"streams\":%d,\"sent\":%u,\"rssi\":%d,\"phy\":%u.%u,\"channel\":%u}",
		       k_uptime_seconds(), st.width, st.height, st.qs, st.fps_x10 / 10,
		       st.fps_x10 % 10, st.interval_us, st.jitter_us, st.frame_avg, st.frame_max,
		       st.frames, st.bytes, st.dropped, st.snapshots, st.snap_size,
		       st.snap_pause_ms, (int)atomic_get(&streams), (uint32_t)atomic_get(&sent),
		       rssi, phy / 10, phy % 10, channel);

	snprintf(head, sizeof(head),
		 "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
		 "Cache-Control: no-store\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
		 len);
	if (send_str(fd, head) == 0) {
		send_all(fd, body, len);
	}
}

static void serve_stream(int fd)
{
	struct video_buffer *vbuf;
	uint32_t seq = 0;
	char head[96];
	size_t len;
	int ret;

	if (atomic_inc(&streams) >= MAX_STREAMS) {
		send_status(fd, "503 Service Unavailable");
		goto out;
	}

	vbuf = video_buffer_alloc(STREAM_FRAME_MAX, K_NO_WAIT);
	if (vbuf == NULL) {
		send_status(fd, "503 Service Unavailable");
		goto out;
	}

	ret = send_str(fd, "HTTP/1.1 200 OK\r\n"
			   "Content-Type: multipart/x-mixed-replace; boundary=" BOUNDARY "\r\n"
			   "Cache-Control: no-cache\r\nConnection: close\r\n\r\n");

	while (ret == 0) {
		ret = camera_frame_copy(vbuf->buffer, vbuf->size, &len, &seq, K_SECONDS(5));
		if (ret < 0) {
			LOG_WRN("No frame for 5 s");
			ret = 0;
			continue;
		}

		snprintf(head, sizeof(head),
			 "--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
			 len);
		ret = send_str(fd, head);
		if (ret == 0) {
			ret = send_all(fd, vbuf->buffer, len);
		}
		if (ret == 0) {
			ret = send_str(fd, "\r\n");
		}
	}

	video_buffer_release(vbuf);
out:
	atomic_dec(&streams);
}

static void serve_snapshot(int fd)
{
	const uint8_t *data;
	char head[192];
	size_t len;
	int ret;

	ret = camera_snapshot(&data, &len);
	if (ret < 0) {
		send_status(fd, "500 Internal Server Error");
		return;
	}

	snprintf(head, sizeof(head),
		 "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n"
		 "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
		 len);
	if (send_str(fd, head) == 0) {
		send_all(fd, data, len);
	}

	camera_snapshot_release();
}

static void serve(int fd)
{
	char req[REQUEST_SIZE];
	size_t used = 0;
	char *path;
	char *end;
	ssize_t ret;

	/* Only the request line matters, read until the end of the headers */
	do {
		ret = zsock_recv(fd, req + used, sizeof(req) - 1 - used, 0);
		if (ret <= 0) {
			return;
		}

		used += ret;
		req[used] = '\0';
	} while (strstr(req, "\r\n\r\n") == NULL && used < sizeof(req) - 1);

	if (strncmp(req, "GET ", 4) != 0) {
		send_status(fd, "405 Method Not Allowed");
		return;
	}

	path = req + 4;
	end = strpbrk(path, " ?\r\n");
	if (end == NULL) {
		send_status(fd, "400 Bad Request");
		return;
	}
	*end = '\0';

	LOG_DBG("GET %s", path);

	if (strcmp(path, "/") == 0) {
		serve_index(fd);
	} else if (strcmp(path, "/stream") == 0) {
		serve_stream(fd);
	} else if (strcmp(path, "/stats") == 0) {
		serve_stats(fd);
	} else if (strcmp(path, "/snapshot.jpg") == 0) {
		serve_snapshot(fd);
	} else {
		send_status(fd, "404 Not Found");
	}
}

static void worker_run(void *p1, void *p2, void *p3)
{
	struct timeval timeout = {.tv_sec = 10};
	int fd;

	for (;;) {
		k_msgq_get(&client_q, &fd, K_FOREVER);

		zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_SNDTIMEO, &timeout,
				 sizeof(timeout));
		zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_RCVTIMEO, &timeout,
				 sizeof(timeout));
		serve(fd);
		zsock_close(fd);
	}
}

static void accept_run(void *p1, void *p2, void *p3)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_port = net_htons(HTTP_PORT),
	};
	struct timeval busy_timeout = {.tv_sec = 1};
	int one = 1;
	int server;
	int fd;

	server = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (server < 0) {
		LOG_ERR("Cannot create the server socket (%d)", errno);
		return;
	}

	zsock_setsockopt(server, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &one, sizeof(one));

	if (zsock_bind(server, (struct net_sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(server, WORKERS) < 0) {
		LOG_ERR("Cannot listen on port %d (%d)", HTTP_PORT, errno);
		zsock_close(server);
		return;
	}

	for (;;) {
		fd = zsock_accept(server, NULL, NULL);
		if (fd < 0) {
			LOG_ERR("Accept failed (%d)", errno);
			continue;
		}

		if (k_msgq_put(&client_q, &fd, K_NO_WAIT) < 0) {
			zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_SNDTIMEO, &busy_timeout,
					 sizeof(busy_timeout));
			send_status(fd, "503 Service Unavailable");
			zsock_close(fd);
		}
	}
}

int http_start(void)
{
	for (int i = 0; i < WORKERS; i++) {
		k_thread_create(&workers[i], worker_stacks[i], STACK_SIZE, worker_run, NULL, NULL,
				NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
		k_thread_name_set(&workers[i], "http_worker");
	}

	k_thread_create(&accept_thread, accept_stack, K_THREAD_STACK_SIZEOF(accept_stack),
			accept_run, NULL, NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
	k_thread_name_set(&accept_thread, "http");

	return 0;
}
