/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "zb_common.h"
#include "zcl_common.h"
#include "zcl_basic.h"
#include "zcl_tempMeasure.h"
#include "zcl_relHumidityMeasure.h"
#include "zcl_powerCfg.h"

#define ENDPOINT                 1
#define HA_PROFILE_ID            0x0104
#define HA_TEMP_SENSOR_DEVICE_ID 0x0302

#define CZIGBEECLUSTER_SIZE      72
#define REPORT_INTERVAL_SEC      30
#define REPORT_INITIAL_DELAY_SEC 5
#define RETRY_INTERVAL_MS        5000
#define INIT_COUNTDOWN_SEC       3

#define CLUSTER_OPT_SERVER       1U
#define ZCL_ADDR_MODE_SHORT      2U
#define COORDINATOR_SHORT_ADDR   0x0000
#define COORDINATOR_EP           1

#define ZCL_VERSION_VAL          3U
#define APP_VERSION_VAL          1U
#define STACK_VERSION_VAL        2U
#define HW_VERSION_VAL           1U
#define POWER_SOURCE_BATTERY     3U

#define TEMP_MIN_MEASURED        (-4000)
#define TEMP_MAX_MEASURED        12500
#define HUM_MIN_MEASURED         0
#define HUM_MAX_MEASURED         10000
#define BATTERY_VOLTAGE_30       30

#define FAKE_TEMP                2250
#define FAKE_HUM                 4500
#define FAKE_BATT_PCT            200

extern void _ZN14CZigBeeClusterC1ER28CZigBeeFoundationApplicationjj(
	void *self, void *app, unsigned int cluster_id, unsigned int options);
extern void _ZN21CMyDynamicApplication10AddClusterEP14CZigBeeCluster(
	void *self, void *cluster);
extern uint8_t zb_findAppCntByEp(uint8_t ep);
extern uint32_t dyClustApp[];

static int16_t zcl_temp = FAKE_TEMP;
static uint16_t zcl_hum = FAKE_HUM;
static uint8_t zcl_batt_pct = FAKE_BATT_PCT;
static struct k_work_delayable report_work;
static volatile bool joined;

static void *create_cluster(void *app, uint16_t cluster_id)
{
	void *cluster = calloc(1, CZIGBEECLUSTER_SIZE);

	if (!cluster) {
		return NULL;
	}
	_ZN14CZigBeeClusterC1ER28CZigBeeFoundationApplicationjj(
		cluster, app, cluster_id, CLUSTER_OPT_SERVER);
	_ZN21CMyDynamicApplication10AddClusterEP14CZigBeeCluster(
		app, cluster);
	return cluster;
}

static void register_clusters(uint8_t ep)
{
	uint8_t idx = zb_findAppCntByEp(ep);

	if (idx == 0xFF) {
		return;
	}

	void *app = (void *)(uintptr_t)dyClustApp[idx * 2];

	create_cluster(app, ZCL_CLUST_BASIC);
	create_cluster(app, ZCL_CLUST_PWR_CFG);
	create_cluster(app, ZCL_CLUST_TEMP_MEASURE);
	create_cluster(app, ZCL_CLUST_REL_HUM_MEASURE);

	printf("clusters registered on ep %d\n", ep);
}

static uint8_t resp_buf[128];

static int put_attr_u8(uint8_t *buf, uint16_t attr_id, uint8_t val)
{
	buf[0] = attr_id & 0xFF;
	buf[1] = attr_id >> 8;
	buf[2] = ZCL_STATUS_SUCCESS;
	buf[3] = ZCL_ATTR_TYPE_U8;
	buf[4] = val;
	return 5;
}

static int put_attr_enum8(uint8_t *buf, uint16_t attr_id, uint8_t val)
{
	buf[0] = attr_id & 0xFF;
	buf[1] = attr_id >> 8;
	buf[2] = ZCL_STATUS_SUCCESS;
	buf[3] = ZCL_ATTR_TYPE_8BITENUM;
	buf[4] = val;
	return 5;
}

static int put_attr_s16(uint8_t *buf, uint16_t attr_id, int16_t val)
{
	buf[0] = attr_id & 0xFF;
	buf[1] = attr_id >> 8;
	buf[2] = ZCL_STATUS_SUCCESS;
	buf[3] = ZCL_ATTR_TYPE_S16;
	buf[4] = val & 0xFF;
	buf[5] = (val >> 8) & 0xFF;
	return 6;
}

static int put_attr_u16(uint8_t *buf, uint16_t attr_id, uint16_t val)
{
	buf[0] = attr_id & 0xFF;
	buf[1] = attr_id >> 8;
	buf[2] = ZCL_STATUS_SUCCESS;
	buf[3] = ZCL_ATTR_TYPE_U16;
	buf[4] = val & 0xFF;
	buf[5] = (val >> 8) & 0xFF;
	return 6;
}

static int put_attr_charstr(uint8_t *buf, uint16_t attr_id, const char *str)
{
	uint8_t len = strlen(str);

	buf[0] = attr_id & 0xFF;
	buf[1] = attr_id >> 8;
	buf[2] = ZCL_STATUS_SUCCESS;
	buf[3] = ZCL_ATTR_TYPE_CHAR_STRING;
	buf[4] = len;
	memcpy(&buf[5], str, len);
	return 5 + len;
}

static int put_attr_unsup(uint8_t *buf, uint16_t attr_id)
{
	buf[0] = attr_id & 0xFF;
	buf[1] = attr_id >> 8;
	buf[2] = ZCL_STATUS_UNSUP_ATTR;
	return 3;
}

static int build_read_attr_resp_basic(const uint8_t *req, uint8_t req_len,
				      uint8_t *out)
{
	int pos = 0;

	for (int i = 0; i + 1 < req_len; i += 2) {
		uint16_t attr = req[i] | (req[i + 1] << 8);

		switch (attr) {
		case ZCL_ATTR_ZCL_VERSION:
			pos += put_attr_u8(out + pos, attr, ZCL_VERSION_VAL);
			break;
		case ZCL_ATTR_APPLICATION_VERSION:
			pos += put_attr_u8(out + pos, attr, APP_VERSION_VAL);
			break;
		case ZCL_ATTR_STACK_VERSION:
			pos += put_attr_u8(out + pos, attr, STACK_VERSION_VAL);
			break;
		case ZCL_ATTR_HW_VERSION:
			pos += put_attr_u8(out + pos, attr, HW_VERSION_VAL);
			break;
		case ZCL_ATTR_MANUFACTURER_NAME:
			pos += put_attr_charstr(out + pos, attr, "ThirdReality");
			break;
		case ZCL_ATTR_MODE_IDENTIFIER:
			pos += put_attr_charstr(out + pos, attr, "3RSL-TNH01");
			break;
		case ZCL_ATTR_POWER_SOURCE:
			pos += put_attr_enum8(out + pos, attr, POWER_SOURCE_BATTERY);
			break;
		default:
			pos += put_attr_unsup(out + pos, attr);
			break;
		}
	}
	return pos;
}

static int build_read_attr_resp_temp(const uint8_t *req, uint8_t req_len,
				     uint8_t *out)
{
	int pos = 0;

	for (int i = 0; i + 1 < req_len; i += 2) {
		uint16_t attr = req[i] | (req[i + 1] << 8);

		switch (attr) {
		case ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE:
			pos += put_attr_s16(out + pos, attr, zcl_temp);
			break;
		case ZCL_ATTR_TEMPERATURE_MEASUREMENT_MIN_MEASURED_VALUE:
			pos += put_attr_s16(out + pos, attr, TEMP_MIN_MEASURED);
			break;
		case ZCL_ATTR_TEMPERATURE_MEASUREMENT_MAX_MEASURED_VALUE:
			pos += put_attr_s16(out + pos, attr, TEMP_MAX_MEASURED);
			break;
		default:
			pos += put_attr_unsup(out + pos, attr);
			break;
		}
	}
	return pos;
}

static int build_read_attr_resp_humidity(const uint8_t *req, uint8_t req_len,
					 uint8_t *out)
{
	int pos = 0;

	for (int i = 0; i + 1 < req_len; i += 2) {
		uint16_t attr = req[i] | (req[i + 1] << 8);

		switch (attr) {
		case ZCL_ATTR_RELATIVE_HUMIDITY_MEASUREMENT_MEASURED_VALUE:
			pos += put_attr_u16(out + pos, attr, zcl_hum);
			break;
		case ZCL_ATTR_RELATIVE_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE:
			pos += put_attr_u16(out + pos, attr, HUM_MIN_MEASURED);
			break;
		case ZCL_ATTR_RELATIVE_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE:
			pos += put_attr_u16(out + pos, attr, HUM_MAX_MEASURED);
			break;
		default:
			pos += put_attr_unsup(out + pos, attr);
			break;
		}
	}
	return pos;
}

static int build_read_attr_resp_power(const uint8_t *req, uint8_t req_len,
				      uint8_t *out)
{
	int pos = 0;

	for (int i = 0; i + 1 < req_len; i += 2) {
		uint16_t attr = req[i] | (req[i + 1] << 8);

		switch (attr) {
		case ZCL_ATTR_POWERCONFIG_BATTERY_PERCENTAGE_REMAINING:
			pos += put_attr_u8(out + pos, attr, zcl_batt_pct);
			break;
		case ZCL_ATTR_POWERCONFIG_BATTERY_VOLTAGE:
			pos += put_attr_u8(out + pos, attr, BATTERY_VOLTAGE_30);
			break;
		default:
			pos += put_attr_unsup(out + pos, attr);
			break;
		}
	}
	return pos;
}

static void send_zcl_response(struct _zclIndication *ind,
			       uint8_t cmd_id, uint8_t *payload, int len)
{
	struct _zclCommand cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.dstAddrMode = ZCL_ADDR_MODE_SHORT;
	cmd.dstAddr.shortAddr = ind->srcAddr.shortAddr;
	cmd.dstEp = ind->srcEp;
	cmd.srcEp = ind->dstEp;
	cmd.profileId = HA_PROFILE_ID;
	cmd.clusterId = ind->clusterId;
	cmd.frameCtrl.bf.frameType = ZCL_FRAME_TYPE_CMD_GLOBAL;
	cmd.frameCtrl.bf.dir = ZCL_CLUST_DIR_SERVER_TO_CLIENT;
	cmd.frameCtrl.bf.disableDftResp = 1;
	cmd.seqNum = ind->seqNum;
	cmd.cmdId = cmd_id;
	cmd.payloadLen = len;
	cmd.payload = payload;
	zcl_sendCommand(&cmd, true);
}

static bool zcl_indication_handler(struct _zclIndication *ind)
{
	int resp_len = 0;

	printf("ZCL: cmd=0x%02x clust=0x%04x src=0x%04x\n",
	       ind->cmdId, ind->clusterId, ind->srcAddr.shortAddr);

	if (ind->cmdId == ZCL_CMD_READ_ATTR) {
		switch (ind->clusterId) {
		case ZCL_CLUST_BASIC:
			resp_len = build_read_attr_resp_basic(
				ind->payload, ind->payloadLen, resp_buf);
			break;
		case ZCL_CLUST_TEMP_MEASURE:
			resp_len = build_read_attr_resp_temp(
				ind->payload, ind->payloadLen, resp_buf);
			break;
		case ZCL_CLUST_REL_HUM_MEASURE:
			resp_len = build_read_attr_resp_humidity(
				ind->payload, ind->payloadLen, resp_buf);
			break;
		case ZCL_CLUST_PWR_CFG:
			resp_len = build_read_attr_resp_power(
				ind->payload, ind->payloadLen, resp_buf);
			break;
		default:
			return true;
		}

		if (resp_len > 0) {
			send_zcl_response(ind, ZCL_CMD_READ_ATTR_RESP,
					  resp_buf, resp_len);
		}
		return false;
	}

	if (ind->cmdId == ZCL_CMD_CONFIG_REPORT) {
		uint8_t success = ZCL_STATUS_SUCCESS;

		send_zcl_response(ind, ZCL_CMD_CONFIG_REPORT_RESP,
				  &success, 1);
		return false;
	}

	return true;
}

static void send_report(uint16_t cluster_id, uint16_t attr_id,
			uint8_t data_type, void *data, uint8_t data_len)
{
	struct _zclTxParam aps;
	struct _zclReportAttrReportRec rec;

	memset(&aps, 0, sizeof(aps));
	aps.dstAddrMode = ZCL_ADDR_MODE_SHORT;
	aps.dstAddr.shortAddr = COORDINATOR_SHORT_ADDR;
	aps.dstEp = COORDINATOR_EP;
	aps.srcEp = ENDPOINT;
	aps.profileId = HA_PROFILE_ID;
	aps.dir = ZCL_CLUST_DIR_SERVER_TO_CLIENT;

	rec.attrId = attr_id;
	rec.attrDataType = data_type;
	rec.attrData = data;

	zcl_sendReportAttr(&aps, cluster_id, 1, &rec, NULL);
}

static void report_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	zcl_temp = FAKE_TEMP;
	zcl_hum = FAKE_HUM;
	zcl_batt_pct = FAKE_BATT_PCT;

	printf("report: t=%d.%02d h=%d.%02d b=%d%%\n",
	       zcl_temp / 100, abs(zcl_temp) % 100,
	       zcl_hum / 100, zcl_hum % 100, zcl_batt_pct / 2);

	send_report(ZCL_CLUST_TEMP_MEASURE,
		    ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE,
		    ZCL_ATTR_TYPE_S16, &zcl_temp, 2);

	send_report(ZCL_CLUST_REL_HUM_MEASURE,
		    ZCL_ATTR_RELATIVE_HUMIDITY_MEASUREMENT_MEASURED_VALUE,
		    ZCL_ATTR_TYPE_U16, &zcl_hum, 2);

	send_report(ZCL_CLUST_PWR_CFG,
		    ZCL_ATTR_POWERCONFIG_BATTERY_PERCENTAGE_REMAINING,
		    ZCL_ATTR_TYPE_U8, &zcl_batt_pct, 1);

	k_work_schedule(&report_work, K_SECONDS(REPORT_INTERVAL_SEC));
}

static zbRet_t zigbee_cb(uint8_t evtId, uint8_t *evtParam)
{
	switch (evtId) {
	case ZB_EVT_STARTUP_COMPLETE: {
		struct _zbStartUpEventParams *p =
			(struct _zbStartUpEventParams *)evtParam;
		printf("STARTUP: status=%d role=%d flags=0x%04x\n",
		       p->status, p->deviceRole, p->startupFlags);
		if (p->status != 0) {
			printf("Join failed (status=%d)\n", p->status);
		} else {
			printf("Joined! short=0x%04x ch=%d panId=0x%04x\n",
			       zb_getShortAddr(), zb_getChannel(), zb_getPanId());
			joined = true;
			k_work_schedule(&report_work,
					K_SECONDS(REPORT_INITIAL_DELAY_SEC));
		}
		break;
	}
	case ZB_EVT_NWK_LEAVE:
		printf("NWK_LEAVE\n");
		break;
	default:
		break;
	}
	return ZB_SUCC;
}

int main(void)
{
	int ret;
	struct _deviceFlags flags = {0};

	k_work_init_delayable(&report_work, report_work_handler);

	for (int i = INIT_COUNTDOWN_SEC; i > 0; i--) {
		printf("init in %d...\n", i);
		k_msleep(MSEC_PER_SEC);
	}

	zb_registerCb(zigbee_cb);

	ret = zb_stackInit();
	printf("stackInit=%d\n", ret);

	zb_setRole(ZB_ROLE_ROUTER);

	ret = zb_registerDevice(ENDPOINT, HA_PROFILE_ID,
				HA_TEMP_SENSOR_DEVICE_ID, flags);
	printf("registerDevice=%d\n", ret);

	register_clusters(ENDPOINT);
	zcl_registerZclIndCb(zcl_indication_handler);

	zb_initDevices();
	zb_setChannelMask(BIT(25));

	ret = zb_start();
	printf("start=%d\n", ret);

	while (1) {
		k_msleep(RETRY_INTERVAL_MS);
		if (!joined) {
			printf("retrying...\n");
			zb_start();
		}
	}
}
