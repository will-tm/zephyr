/*
 * Copyright (c) 2016-2022 Bouffalolab.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * TLV utilities — copied from M1s SDK components/utils/src/utils_tlv_bl.c
 */

#include <stdint.h>
#include <stdbool.h>

#define CFG_ELEMENT_TYPE_SIZE_BOOLEAN         (4)
#define CFG_ELEMENT_TYPE_SIZE_UINT32          (4)
#define UTILS_TLV_BL_ERROR_CODE_BUF_TOO_SMALL (-1)
#define UTILS_TLV_BL_ERROR_CODE_UNKOWN        (-2)

enum CFG_ELEMENT_TYPE {
	CFG_ELEMENT_TYPE_UNKNOWN,
	CFG_ELEMENT_TYPE_BOOLEAN,
	CFG_ELEMENT_TYPE_SINT8,
	CFG_ELEMENT_TYPE_UINT8,
	CFG_ELEMENT_TYPE_SINT16,
	CFG_ELEMENT_TYPE_UINT16,
	CFG_ELEMENT_TYPE_SINT32,
	CFG_ELEMENT_TYPE_UINT32,
	CFG_ELEMENT_TYPE_STRING,
};

int utils_tlv_bl_pack_bool(uint32_t *buf, int buf_sz, bool val)
{
	if (buf_sz < CFG_ELEMENT_TYPE_SIZE_BOOLEAN) {
		return UTILS_TLV_BL_ERROR_CODE_BUF_TOO_SMALL;
	}
	*buf = val;
	return CFG_ELEMENT_TYPE_SIZE_BOOLEAN;
}

int utils_tlv_bl_pack_uint32(uint32_t *buf, int buf_sz, uint32_t val)
{
	if (buf_sz < CFG_ELEMENT_TYPE_SIZE_UINT32) {
		return UTILS_TLV_BL_ERROR_CODE_BUF_TOO_SMALL;
	}
	*buf = val;
	return CFG_ELEMENT_TYPE_SIZE_UINT32;
}

int utils_tlv_bl_unpack_bool(uint32_t *buf, int buf_sz, bool *val)
{
	if (buf_sz < CFG_ELEMENT_TYPE_SIZE_BOOLEAN) {
		return UTILS_TLV_BL_ERROR_CODE_BUF_TOO_SMALL;
	}
	*val = (*buf) ? true : false;
	return CFG_ELEMENT_TYPE_SIZE_BOOLEAN;
}

int utils_tlv_bl_unpack_uint32(uint32_t *buf, int buf_sz, uint32_t *val)
{
	if (buf_sz < CFG_ELEMENT_TYPE_SIZE_UINT32) {
		return UTILS_TLV_BL_ERROR_CODE_BUF_TOO_SMALL;
	}
	*val = *buf;
	return CFG_ELEMENT_TYPE_SIZE_UINT32;
}

int utils_tlv_bl_pack_auto(uint32_t *buf, int buf_sz, uint16_t type, void *arg1)
{
	switch (type) {
	case CFG_ELEMENT_TYPE_BOOLEAN:
		return utils_tlv_bl_pack_bool(buf, buf_sz, *(bool *)arg1 ? true : false);
	case CFG_ELEMENT_TYPE_UINT32:
		return utils_tlv_bl_pack_uint32(buf, buf_sz, *(uint32_t *)arg1);
	default:
		return UTILS_TLV_BL_ERROR_CODE_UNKOWN;
	}
}

int utils_tlv_bl_unpack_auto(uint32_t *buf, int buf_sz, uint16_t type, void *arg1)
{
	switch (type) {
	case CFG_ELEMENT_TYPE_BOOLEAN: {
		bool val = true;
		int ret = utils_tlv_bl_unpack_bool(buf, buf_sz, &val);

		*(bool *)arg1 = val;
		return ret;
	}
	case CFG_ELEMENT_TYPE_UINT32: {
		uint32_t val = 0;
		int ret = utils_tlv_bl_unpack_uint32(buf, buf_sz, &val);

		*(uint32_t *)arg1 = val;
		return ret;
	}
	default:
		return UTILS_TLV_BL_ERROR_CODE_UNKOWN;
	}
}
