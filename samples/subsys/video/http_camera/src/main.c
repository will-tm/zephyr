/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "app.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define WIFI_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)
#define RECONNECT_DELAY K_SECONDS(5)

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;
static struct net_if *iface;

static void wifi_connect(struct k_work *work)
{
	struct wifi_connect_req_params params = {
		.ssid = CONFIG_APP_WIFI_SSID,
		.ssid_length = sizeof(CONFIG_APP_WIFI_SSID) - 1,
		.psk = CONFIG_APP_WIFI_PSK,
		.psk_length = sizeof(CONFIG_APP_WIFI_PSK) - 1,
		.security = WIFI_SECURITY_TYPE_PSK,
		.channel = WIFI_CHANNEL_ANY,
		.band = WIFI_FREQ_BAND_UNKNOWN,
		.mfp = WIFI_MFP_OPTIONAL,
		.timeout = SYS_FOREVER_MS,
	};
	int ret;

	LOG_INF("Connecting to \"%s\"", CONFIG_APP_WIFI_SSID);
	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	if (ret < 0) {
		LOG_ERR("Connect request failed (%d)", ret);
		k_work_reschedule(k_work_delayable_from_work(work), RECONNECT_DELAY);
	}
}

static K_WORK_DELAYABLE_DEFINE(connect_work, wifi_connect);

static void wifi_event(struct net_mgmt_event_callback *cb, uint64_t event, struct net_if *ifc)
{
	const struct wifi_status *status = cb->info;

	if (event == NET_EVENT_WIFI_CONNECT_RESULT && status->status == 0) {
		LOG_INF("Connected");
		return;
	}

	LOG_WRN("%s (%d), retrying", event == NET_EVENT_WIFI_CONNECT_RESULT ?
		"Connection failed" : "Disconnected", status->status);
	k_work_reschedule(&connect_work, RECONNECT_DELAY);
}

static void ipv4_event(struct net_mgmt_event_callback *cb, uint64_t event, struct net_if *ifc)
{
	char addr[NET_IPV4_ADDR_LEN];

	for (int i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		struct net_if_addr *ifaddr = &ifc->config.ip.ipv4->unicast[i].ipv4;

		if (ifaddr->is_used && ifaddr->addr_type == NET_ADDR_DHCP) {
			net_addr_ntop(NET_AF_INET, &ifaddr->address.in_addr, addr, sizeof(addr));
			LOG_INF("Camera at http://%s/", addr);
		}
	}
}

int main(void)
{
	int ret;

	ret = camera_start();
	if (ret < 0) {
		return ret;
	}

	ret = http_start();
	if (ret < 0) {
		return ret;
	}

	iface = net_if_get_wifi_sta();
	if (iface == NULL) {
		LOG_ERR("No WiFi interface");
		return -ENODEV;
	}

	net_mgmt_init_event_callback(&wifi_cb, wifi_event, WIFI_EVENTS);
	net_mgmt_add_event_callback(&wifi_cb);
	net_mgmt_init_event_callback(&ipv4_cb, ipv4_event, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&ipv4_cb);

	net_dhcpv4_start(iface);
	k_work_reschedule(&connect_work, K_NO_WAIT);

	return 0;
}

void wifi_get_status(int *rssi, uint32_t *phy_x10, uint32_t *channel)
{
	struct wifi_iface_status status = {0};

	if (iface == NULL ||
	    net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status)) < 0 ||
	    status.state < WIFI_STATE_ASSOCIATED) {
		*rssi = 0;
		*phy_x10 = 0;
		*channel = 0;
		return;
	}

	*rssi = status.rssi;
	*phy_x10 = (uint32_t)(status.current_phy_tx_rate * 10.0f);
	*channel = status.channel;
}
