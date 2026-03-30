/* WiFi firmware configuration — matches pinevoice SDK build options */
#ifndef WIFI_CFG_H
#define WIFI_CFG_H

#include <stdint.h>
#include <stddef.h>

/* Chip identification — needed by ipc_shared.h and bl_tx.c for TX path */
#define CFG_CHIP_BL808     1
#define CFG_CHIP_BL606P    1

/* Firmware config defines */
#define CFG_TXDESC          4
#define CFG_VIRT_DEV_MAX    2
#define CFG_STA_MAX         5
#define REG_WIFI_REG_BASE   0x24000000

/* Ensure BIT macro is available (needed by cfg80211.h enums) */
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

#undef container_of
#define container_of(ptr, type, member) \
	((type *)((char *)(1 ? (ptr) : &((type *)0)->member) - __builtin_offsetof(type, member)))


/* LWIP types stubbed — Zephyr uses its own net stack */
typedef int err_t;
#define ERR_OK 0

#define PBUF_RAW  0
#define PBUF_REF  1
#define PBUF_POOL 2
struct pbuf {
	void *payload; uint16_t len; uint16_t tot_len;
	struct pbuf *next; uint8_t type; uint8_t flags; uint16_t ref;
};
struct pbuf_custom {
	struct pbuf pbuf;
	void (*custom_free_function)(struct pbuf *p);
};

/* Full ABI-compatible struct netif (matches blob's expected layout) */
typedef int (*netif_input_fn)(struct pbuf *p, struct netif *inp);
typedef int (*netif_output_fn)(struct netif *netif, struct pbuf *p, void *ipaddr);
typedef int (*netif_linkoutput_fn)(struct netif *netif, struct pbuf *p);
typedef void (*netif_status_callback_fn)(struct netif *netif);
typedef void (*dhcp_quick_connect_callback_fn)(struct netif *netif);
typedef int (*netif_igmp_mac_filter_fn)(struct netif *netif, void *group, uint8_t action);
typedef uint32_t ip_addr_t;

struct addr_ext {
	uint8_t arp_for_us_disable;
	dhcp_quick_connect_callback_fn dhcp_qc_callback;
};

struct netif {
	struct netif *next;
	ip_addr_t ip_addr;
	ip_addr_t netmask;
	ip_addr_t gw;
	netif_input_fn input;
	netif_output_fn output;
	netif_linkoutput_fn linkoutput;
	netif_status_callback_fn status_callback;
	netif_status_callback_fn link_callback;
	void *state;
	void *client_data[3];
	const char *hostname;
	uint16_t mtu;
	uint8_t hwaddr[6];
	uint8_t hwaddr_len;
	uint8_t flags;
	char name[2];
	uint8_t num;
	netif_igmp_mac_filter_fn igmp_mac_filter;
	struct pbuf *loop_first;
	struct pbuf *loop_last;
	struct addr_ext addr_ext;
};

/* SDK types not available in Zephyr */
typedef void *hosal_adc_dev_t;
typedef void (*bl_pm_cb_t)(void);
typedef uint32_t BL_TimeOut_t;
typedef uint32_t BL_TickType_t;

/* Power management stubs */
#define PM_MODE_STA_NONE  0
#define PM_MODE_STA_IDLE  1
#define PM_MODE_STA_MESH  2
#define PM_MODE_STA_DOZE  3
#define PM_MODE_STA_COEX  4
#define PM_MODE_STA_DOWN  5
#define PM_MODE_AP_IDLE   6
#define PM_MODE_MAX       7

enum PM_EVEMT { PM_EVEMT_DUMMY };
enum PM_EVENT_ABLE { PM_EVENT_DISABLE, PM_EVENT_ENABLE };
enum PM_LEVEL { PM_LEVEL_ACTIVE };

#endif /* WIFI_CFG_H */

#define EV_WIFI 0
#define CODE_WIFI_ON_AP_STA_ADD 0
#define CODE_WIFI_ON_AP_STA_DEL 0

