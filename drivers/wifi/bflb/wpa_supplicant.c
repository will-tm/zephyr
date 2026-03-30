/*
 * Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal WPA2-PSK supplicant for BL808 WiFi firmware.
 *
 * Implements the IEEE 802.11i 4-way handshake:
 *   M1 (AP→STA) → M2 (STA→AP) → M3 (AP→STA) → M4 (STA→AP)
 *
 * Crypto uses mbedTLS (HMAC-SHA1, AES-128-ECB).
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>

#include <mbedtls/md.h>

#include "bl_defs.h"
#include "bl_tx.h"
#include <supplicant_api.h>

LOG_MODULE_REGISTER(wpa_supp, LOG_LEVEL_ERR);

/* -------- Constants -------- */

#define WPA_NONCE_LEN       32
#define WPA_REPLAY_CTR_LEN  8
#define PMK_LEN             32
#define KCK_LEN             16
#define KEK_LEN             16
#define TK_LEN              16
#define PTK_LEN             (KCK_LEN + KEK_LEN + TK_LEN) /* 48 */
#define SHA1_MAC_LEN        20
#define ETH_P_EAPOL         0x888E
#define EAPOL_VERSION       2
#define IEEE802_1X_TYPE_KEY 3
#define EAPOL_KEY_TYPE_RSN  2

/* Key Info bit definitions (big-endian in frame, host-order here) */
#define WPA_KEY_INFO_TYPE_MASK    0x0007
#define WPA_KEY_INFO_TYPE_AES_SHA 0x0002 /* HMAC-SHA1-128 + AES */
#define WPA_KEY_INFO_KEY_TYPE     0x0008 /* pairwise */
#define WPA_KEY_INFO_INSTALL      0x0040
#define WPA_KEY_INFO_ACK          0x0080
#define WPA_KEY_INFO_MIC          0x0100
#define WPA_KEY_INFO_SECURE       0x0200
#define WPA_KEY_INFO_ENCR         0x1000

/* EAPOL-Key frame header (after ieee802_1x_hdr) */
struct wpa_eapol_key {
	uint8_t type;
	uint8_t key_info[2];   /* big-endian */
	uint8_t key_length[2]; /* big-endian */
	uint8_t replay_counter[8];
	uint8_t key_nonce[32];
	uint8_t key_iv[16];
	uint8_t key_rsc[8];
	uint8_t key_id[8];
	uint8_t key_mic[16];
	uint8_t key_data_length[2]; /* big-endian */
				    /* key_data follows */
} __packed;

#define EAPOL_KEY_HDR_LEN sizeof(struct wpa_eapol_key) /* 95 */

/* IEEE 802.1X header */
struct ieee802_1x_hdr {
	uint8_t version;
	uint8_t type;
	uint8_t length[2]; /* big-endian */
} __packed;

/* WPA SM state */
enum wpa_state {
	WPA_STATE_IDLE,
	WPA_STATE_WAIT_M1,
	WPA_STATE_WAIT_M3,
	WPA_STATE_COMPLETED,
};

struct wpa_sm {
	enum wpa_state state;
	uint8_t pmk[PMK_LEN];
	uint8_t ptk[PTK_LEN]; /* KCK(16) + KEK(16) + TK(16) */
	uint8_t snonce[WPA_NONCE_LEN];
	uint8_t anonce[WPA_NONCE_LEN];
	uint8_t own_addr[ETH_ALEN];
	uint8_t bssid[ETH_ALEN];
	uint8_t replay_counter[WPA_REPLAY_CTR_LEN];
	uint8_t vif_idx;
	uint8_t sta_idx;
	bool ptk_set;
};

static struct wpa_sm sm;

/* Deferred key installation — keys must be installed AFTER the firmware
 * has processed and transmitted M4.  The firmware TX processing is
 * asynchronous (runs in the firmware event loop after we return from
 * the EAPOL RX handler).  If we install keys before M4 is transmitted,
 * the firmware encrypts M4 with CCMP, but the AP expects it unencrypted. */
static struct {
	uint8_t tk[TK_LEN];
	uint8_t gtk[TK_LEN];
	uint8_t gtk_rsc[8];
	uint8_t gtk_idx;
	bool has_gtk;
	uint8_t vif_idx;
	uint8_t sta_idx;
} pending_keys;

static void install_keys_work_handler(struct k_work *work);
static int wpa_send_eapol(struct wpa_sm *s, uint16_t key_info, const uint8_t *key_data,
			  uint16_t key_data_len);
static K_WORK_DELAYABLE_DEFINE(install_keys_dwork, install_keys_work_handler);

static void install_keys_work_handler(struct k_work *work)
{
	int ret;

	/* Step 1: Send M4 (MUST be sent before key install, unencrypted) */
	uint16_t ki = WPA_KEY_INFO_TYPE_AES_SHA | WPA_KEY_INFO_KEY_TYPE | WPA_KEY_INFO_MIC |
		      WPA_KEY_INFO_SECURE;

	ret = wpa_send_eapol(&sm, ki, NULL, 0);
	if (ret != 0) {
		LOG_ERR("M4 send failed: %d", ret);
	} else {
		LOG_INF("M4 sent from work queue");
	}

	/* Step 2: Wait for firmware to transmit M4 over the air */
	k_msleep(100);

	/* Step 3: Install keys */
	ret = bl_wifi_set_sta_key_internal(pending_keys.vif_idx, pending_keys.sta_idx,
					   WIFI_WPA_ALG_CCMP, 0, 1, NULL, 0, pending_keys.tk,
					   TK_LEN, true);
	if (ret != 0) {
		LOG_ERR("PTK install failed: %d", ret);
	}
	sm.ptk_set = true;

	if (pending_keys.has_gtk) {
		LOG_INF("Installing GTK (idx=%u)", pending_keys.gtk_idx);
		ret = bl_wifi_set_sta_key_internal(pending_keys.vif_idx, pending_keys.sta_idx,
						   WIFI_WPA_ALG_CCMP, pending_keys.gtk_idx, 0,
						   pending_keys.gtk_rsc, 6, pending_keys.gtk,
						   TK_LEN, false);
		if (ret != 0) {
			LOG_WRN("GTK install failed: %d", ret);
		}
	}

	bl_wifi_auth_done_internal(pending_keys.sta_idx, 0);
	LOG_INF("Keys installed, auth done signaled");
}

/* RSN IE used in association — shared with wifi_stubs.c */
extern uint8_t wpa_rsn_ie[22];
extern uint8_t wpa_rsn_ie_len;

/* -------- Crypto helpers -------- */

static int hmac_sha1(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len,
		     uint8_t *mac)
{
	const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);

	if (!md) {
		return -1;
	}
	return mbedtls_md_hmac(md, key, key_len, data, data_len, mac);
}

/*
 * PBKDF2-SHA1 — derive PSK from passphrase + SSID.
 * RFC 2898, 4096 iterations, 32-byte output.
 */
static int pbkdf2_sha1(const char *passphrase, const uint8_t *ssid, size_t ssid_len, uint8_t *pmk)
{
	const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
	uint8_t u[SHA1_MAC_LEN], t[SHA1_MAC_LEN];
	uint8_t salt[32 + 4];
	size_t pass_len = strlen(passphrase);

	if (!md || ssid_len > 32) {
		return -1;
	}

	memcpy(salt, ssid, ssid_len);

	/* Two blocks needed: 20 + 12 = 32 bytes */
	for (int block = 1; block <= 2; block++) {
		salt[ssid_len + 0] = 0;
		salt[ssid_len + 1] = 0;
		salt[ssid_len + 2] = 0;
		salt[ssid_len + 3] = (uint8_t)block;

		if (mbedtls_md_hmac(md, (const uint8_t *)passphrase, pass_len, salt, ssid_len + 4,
				    u) != 0) {
			return -1;
		}
		memcpy(t, u, SHA1_MAC_LEN);

		for (int i = 1; i < 4096; i++) {
			if (mbedtls_md_hmac(md, (const uint8_t *)passphrase, pass_len, u,
					    SHA1_MAC_LEN, u) != 0) {
				return -1;
			}
			for (int j = 0; j < SHA1_MAC_LEN; j++) {
				t[j] ^= u[j];
			}
		}

		size_t off = (block - 1) * SHA1_MAC_LEN;
		size_t copy = (block == 2) ? 12 : SHA1_MAC_LEN;

		memcpy(pmk + off, t, copy);
	}

	return 0;
}

/*
 * SHA1-PRF — PRF-384 for PTK derivation (IEEE 802.11i-2004 8.5.1.1).
 *
 * PRF-X(K, A, B) = HMAC-SHA1(K, A || 0x00 || B || counter)
 */
static int sha1_prf(const uint8_t *key, size_t key_len, const char *label, const uint8_t *data,
		    size_t data_len, uint8_t *output, size_t output_len)
{
	size_t label_len = strlen(label);
	/* label + 0x00 + data + 1 byte counter */
	uint8_t input[128];
	size_t input_len = label_len + 1 + data_len + 1;
	uint8_t hash[SHA1_MAC_LEN];
	size_t pos = 0;
	uint8_t counter = 0;

	if (input_len > sizeof(input)) {
		return -1;
	}

	memcpy(input, label, label_len);
	input[label_len] = 0;
	memcpy(input + label_len + 1, data, data_len);

	while (pos < output_len) {
		input[input_len - 1] = counter;
		if (hmac_sha1(key, key_len, input, input_len, hash) != 0) {
			return -1;
		}
		size_t copy = output_len - pos;

		if (copy > SHA1_MAC_LEN) {
			copy = SHA1_MAC_LEN;
		}
		memcpy(output + pos, hash, copy);
		pos += copy;
		counter++;
	}

	return 0;
}

/*
 * Derive PTK from PMK, addresses, and nonces.
 *
 * data = Min(AA,SA) || Max(AA,SA) || Min(ANonce,SNonce) || Max(ANonce,SNonce)
 */
static int wpa_derive_ptk(struct wpa_sm *s)
{
	uint8_t data[2 * ETH_ALEN + 2 * WPA_NONCE_LEN];
	const uint8_t *addr_lo, *addr_hi;
	const uint8_t *nonce_lo, *nonce_hi;

	if (memcmp(s->own_addr, s->bssid, ETH_ALEN) < 0) {
		addr_lo = s->own_addr;
		addr_hi = s->bssid;
	} else {
		addr_lo = s->bssid;
		addr_hi = s->own_addr;
	}

	if (memcmp(s->snonce, s->anonce, WPA_NONCE_LEN) < 0) {
		nonce_lo = s->snonce;
		nonce_hi = s->anonce;
	} else {
		nonce_lo = s->anonce;
		nonce_hi = s->snonce;
	}

	memcpy(data, addr_lo, ETH_ALEN);
	memcpy(data + ETH_ALEN, addr_hi, ETH_ALEN);
	memcpy(data + 2 * ETH_ALEN, nonce_lo, WPA_NONCE_LEN);
	memcpy(data + 2 * ETH_ALEN + WPA_NONCE_LEN, nonce_hi, WPA_NONCE_LEN);

	return sha1_prf(s->pmk, PMK_LEN, "Pairwise key expansion", data, sizeof(data), s->ptk,
			PTK_LEN);
}

/*
 * AES Key Unwrap (RFC 3394) — decrypt GTK from M3 key data.
 */
/* AES-128 decrypt block — FIPS 197, only used for RFC 3394 key unwrap.
 * mbedTLS 4.x moved AES behind PSA crypto which adds ~7KB RAM. */
static void aes128_decrypt_block(const uint8_t rk[176], const uint8_t in[16], uint8_t out[16])
{
	static const uint8_t inv_sbox[256] = {
		0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
		0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
		0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
		0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
		0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
		0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
		0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
		0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
		0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
		0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
		0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
		0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
		0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
		0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
		0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
		0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d};
	static const uint8_t imix[4][4] = {
		{0x0e,0x0b,0x0d,0x09},{0x09,0x0e,0x0b,0x0d},
		{0x0d,0x09,0x0e,0x0b},{0x0b,0x0d,0x09,0x0e}};
	uint8_t s[16];
	memcpy(s, in, 16);
	for (int i = 0; i < 16; i++) s[i] ^= rk[160 + i];
	for (int r = 9; r >= 0; r--) {
		uint8_t t;
		t=s[13]; s[13]=s[9]; s[9]=s[5]; s[5]=s[1]; s[1]=t;
		t=s[2]; s[2]=s[10]; s[10]=t; t=s[6]; s[6]=s[14]; s[14]=t;
		t=s[3]; s[3]=s[7]; s[7]=s[11]; s[11]=s[15]; s[15]=t;
		for (int i = 0; i < 16; i++) s[i] = inv_sbox[s[i]];
		for (int i = 0; i < 16; i++) s[i] ^= rk[r*16 + i];
		if (r > 0) {
			for (int c = 0; c < 4; c++) {
				uint8_t a[4], o[4];
				memcpy(a, s+4*c, 4);
				for (int i = 0; i < 4; i++) {
					uint8_t v = 0;
					for (int j = 0; j < 4; j++) {
						uint8_t m = imix[i][j], x = a[j], p = 0;
						for (int b = 0; b < 8; b++) {
							if (m & 1) p ^= x;
							uint8_t hi = x & 0x80;
							x <<= 1; if (hi) x ^= 0x1b;
							m >>= 1;
						}
						v ^= p;
					}
					o[i] = v;
				}
				memcpy(s+4*c, o, 4);
			}
		}
	}
	memcpy(out, s, 16);
}

static void aes128_expand_key(const uint8_t key[16], uint8_t rk[176])
{
	static const uint8_t sbox[256] = {
		0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
		0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
		0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
		0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
		0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
		0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
		0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
		0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
		0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
		0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
		0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
		0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
		0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
		0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
		0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
		0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16};
	static const uint8_t rc[10] = {1,2,4,8,16,32,64,128,27,54};
	memcpy(rk, key, 16);
	for (int i = 4; i < 44; i++) {
		uint8_t t[4];
		memcpy(t, rk + (i-1)*4, 4);
		if (i % 4 == 0) {
			uint8_t tmp = t[0];
			t[0] = sbox[t[1]] ^ rc[i/4-1]; t[1] = sbox[t[2]];
			t[2] = sbox[t[3]]; t[3] = sbox[tmp];
		}
		for (int j = 0; j < 4; j++) rk[i*4+j] = rk[(i-4)*4+j] ^ t[j];
	}
}

static int aes_unwrap(const uint8_t *kek, size_t kek_len, int n, const uint8_t *cipher,
		      uint8_t *plain)
{
	uint8_t rk[176], a[8], b[16];
	int i, j;

	if (n < 1 || kek_len != 16) {
		return -1;
	}

	aes128_expand_key(kek, rk);

	memcpy(a, cipher, 8);
	memcpy(plain, cipher + 8, n * 8);

	for (j = 5; j >= 0; j--) {
		for (i = n; i >= 1; i--) {
			uint32_t t = (uint32_t)(n * j + i);
			a[7] ^= (uint8_t)(t & 0xff);
			a[6] ^= (uint8_t)((t >> 8) & 0xff);
			a[5] ^= (uint8_t)((t >> 16) & 0xff);
			a[4] ^= (uint8_t)((t >> 24) & 0xff);
			memcpy(b, a, 8);
			memcpy(b + 8, plain + (i - 1) * 8, 8);
			aes128_decrypt_block(rk, b, b);
			memcpy(a, b, 8);
			memcpy(plain + (i - 1) * 8, b + 8, 8);
		}
	}

	/* Check integrity: A must be 0xA6A6A6A6A6A6A6A6 */
	for (i = 0; i < 8; i++) {
		if (a[i] != 0xA6) {
			LOG_ERR("AES unwrap integrity check failed");
			return -1;
		}
	}

	return 0;
}

/* -------- Helpers -------- */

/* Use Zephyr sys_get_be16 / sys_put_be16 / sys_put_be64 from byteorder.h.
 * Aliases for brevity in crypto code. */
#define be16(p)        sys_get_be16(p)
#define put_be16(p, v) sys_put_be16(v, p)
#define put_be64(p, v) sys_put_be64(v, p)

/*
 * Compute EAPOL-Key MIC over the full EAPOL frame.
 * MIC field must be zeroed before calling.
 */
static int wpa_eapol_key_mic(const uint8_t *kck, const uint8_t *frame, size_t frame_len,
			     uint8_t *mic_out)
{
	uint8_t hash[SHA1_MAC_LEN];

	if (hmac_sha1(kck, KCK_LEN, frame, frame_len, hash) != 0) {
		return -1;
	}
	/* MIC is first 16 bytes of HMAC-SHA1 output */
	memcpy(mic_out, hash, 16);
	return 0;
}

/*
 * Build and send an EAPOL-Key frame.
 */
static int wpa_send_eapol(struct wpa_sm *s, uint16_t key_info, const uint8_t *key_data,
			  uint16_t key_data_len)
{
	/* Build full Ethernet + EAPOL frame */
	uint16_t eapol_body_len = EAPOL_KEY_HDR_LEN + key_data_len;
	uint16_t total_len = sizeof(struct ethhdr) + sizeof(struct ieee802_1x_hdr) + eapol_body_len;
	uint8_t frame[256];

	if (total_len > sizeof(frame)) {
		LOG_ERR("EAPOL frame too large: %u", total_len);
		return -1;
	}

	memset(frame, 0, total_len);

	/* Ethernet header */
	struct ethhdr *eth = (struct ethhdr *)frame;

	memcpy(eth->h_dest, s->bssid, ETH_ALEN);
	memcpy(eth->h_source, s->own_addr, ETH_ALEN);
	eth->h_proto = sys_cpu_to_be16(ETH_P_EAPOL);

	/* IEEE 802.1X header */
	struct ieee802_1x_hdr *hdr = (struct ieee802_1x_hdr *)(frame + sizeof(struct ethhdr));
	hdr->version = EAPOL_VERSION;
	hdr->type = IEEE802_1X_TYPE_KEY;
	put_be16(hdr->length, eapol_body_len);

	/* EAPOL-Key body */
	struct wpa_eapol_key *key = (struct wpa_eapol_key *)(frame + sizeof(struct ethhdr) +
							     sizeof(struct ieee802_1x_hdr));
	key->type = EAPOL_KEY_TYPE_RSN;
	put_be16(key->key_info, key_info);
	/* RSN M2: key_length must be 0 (IEEE 802.11i-2004 8.5.3.2).
	 * M4: also 0.  Only M1/M3 (from AP) carry the actual key length. */
	put_be16(key->key_length, 0);
	memcpy(key->replay_counter, s->replay_counter, WPA_REPLAY_CTR_LEN);
	memcpy(key->key_nonce, s->snonce, WPA_NONCE_LEN);
	put_be16(key->key_data_length, key_data_len);

	if (key_data && key_data_len > 0) {
		memcpy((uint8_t *)key + EAPOL_KEY_HDR_LEN, key_data, key_data_len);
	}

	/* Compute MIC over the full EAPOL frame (MIC field is already zero) */
	if (key_info & WPA_KEY_INFO_MIC) {
		uint8_t *eapol_start = frame + sizeof(struct ethhdr);
		size_t eapol_len = sizeof(struct ieee802_1x_hdr) + eapol_body_len;

		if (wpa_eapol_key_mic(s->ptk, eapol_start, eapol_len, key->key_mic) != 0) {
			LOG_ERR("MIC computation failed");
			return -1;
		}
	}

	LOG_INF("TX EAPOL: key_info=0x%04x data_len=%u total=%u", key_info, key_data_len,
		total_len);
	/* Dump full EAPOL frame (after eth header) for MIC verification */
	uint8_t *eapol_dbg = frame + sizeof(struct ethhdr);
	size_t eapol_dbg_len = sizeof(struct ieee802_1x_hdr) + eapol_body_len;

	LOG_HEXDUMP_DBG(eapol_dbg, eapol_dbg_len, "TX EAPOL full");

	return bl_output_raw(frame, total_len, s->vif_idx, s->sta_idx);
}

/*
 * Process Message 1 of 4-way handshake.
 */
static int wpa_process_m1(struct wpa_sm *s, const struct wpa_eapol_key *key, size_t key_data_len)
{
	/* Extract ANonce from M1 */
	memcpy(s->anonce, key->key_nonce, WPA_NONCE_LEN);
	memcpy(s->replay_counter, key->replay_counter, WPA_REPLAY_CTR_LEN);

	/* Generate random SNonce */
	sys_rand_get(s->snonce, WPA_NONCE_LEN);

	LOG_INF("M1: ANonce received, deriving PTK");
	LOG_HEXDUMP_DBG(s->anonce, WPA_NONCE_LEN, "ANonce");
	LOG_HEXDUMP_DBG(s->own_addr, ETH_ALEN, "own_addr");
	LOG_HEXDUMP_DBG(s->bssid, ETH_ALEN, "bssid");

	/* Derive PTK */
	if (wpa_derive_ptk(s) != 0) {
		LOG_ERR("PTK derivation failed");
		return -1;
	}
	LOG_HEXDUMP_DBG(s->ptk, KCK_LEN, "KCK");
	LOG_HEXDUMP_DBG(s->snonce, WPA_NONCE_LEN, "SNonce");

	/* Send M2: key_info = MIC | Key Type | Version(AES-SHA1) */
	uint16_t ki = WPA_KEY_INFO_TYPE_AES_SHA | WPA_KEY_INFO_KEY_TYPE | WPA_KEY_INFO_MIC;

	/* Include RSN IE as key data */
	int ret = wpa_send_eapol(s, ki, wpa_rsn_ie, wpa_rsn_ie_len);

	if (ret == 0) {
		s->state = WPA_STATE_WAIT_M3;
		LOG_INF("M2 sent, waiting for M3");
	}

	return ret;
}

/*
 * Parse KDE (Key Data Encapsulation) entries from M3 key data.
 * Returns GTK pointer and length, or NULL if not found.
 */
static const uint8_t *wpa_find_gtk_kde(const uint8_t *data, size_t len, size_t *gtk_len,
				       uint8_t *key_idx)
{
	const uint8_t *pos = data;
	const uint8_t *end = data + len;

	while (pos + 2 <= end) {
		uint8_t type = pos[0];
		uint8_t elen = pos[1];

		if (pos + 2 + elen > end) {
			break;
		}

		/* KDE: type=0xDD, OUI=00:0F:AC, data_type */
		if (type == 0xDD && elen >= 6 && pos[2] == 0x00 && pos[3] == 0x0F &&
		    pos[4] == 0xAC && pos[5] == 0x01) {
			/* GTK KDE: OUI type 1 */
			/* pos[6] = key ID (bits 0-1) | Tx (bit 2) */
			/* pos[7] = reserved */
			/* pos[8..] = GTK */
			*key_idx = pos[6] & 0x03;
			*gtk_len = elen - 6;
			return pos + 8; /* skip type+len+OUI+type+keyinfo+rsvd */
		}

		pos += 2 + elen;
	}

	return NULL;
}

/*
 * Process Message 3 of 4-way handshake.
 */
static int wpa_process_m3(struct wpa_sm *s, const struct wpa_eapol_key *key,
			  const uint8_t *eapol_frame, size_t eapol_len)
{
	uint16_t key_info = be16(key->key_info);
	uint16_t key_data_len = be16(key->key_data_length);
	const uint8_t *key_data = (const uint8_t *)key + EAPOL_KEY_HDR_LEN;
	const uint8_t *kck = s->ptk;
	const uint8_t *kek = s->ptk + KCK_LEN;
	const uint8_t *tk = s->ptk + KCK_LEN + KEK_LEN;

	/* Verify ANonce matches M1 */
	if (memcmp(key->key_nonce, s->anonce, WPA_NONCE_LEN) != 0) {
		LOG_ERR("M3: ANonce mismatch");
		return -1;
	}

	/* Verify MIC */
	uint8_t mic_calc[16];
	uint8_t mic_save[16];
	/* Make a mutable copy of the EAPOL frame for MIC verification */
	uint8_t eapol_copy[512];

	if (eapol_len > sizeof(eapol_copy)) {
		LOG_ERR("M3: EAPOL frame too large for MIC check");
		return -1;
	}
	memcpy(eapol_copy, eapol_frame, eapol_len);

	/* Find MIC field in copy and zero it */
	struct wpa_eapol_key *key_copy =
		(struct wpa_eapol_key *)(eapol_copy + sizeof(struct ieee802_1x_hdr));
	memcpy(mic_save, key_copy->key_mic, 16);
	memset(key_copy->key_mic, 0, 16);

	if (wpa_eapol_key_mic(kck, eapol_copy, eapol_len, mic_calc) != 0) {
		LOG_ERR("M3: MIC computation failed");
		return -1;
	}

	if (memcmp(mic_save, mic_calc, 16) != 0) {
		LOG_ERR("M3: MIC verification FAILED");
		LOG_HEXDUMP_ERR(mic_save, 16, "expected");
		LOG_HEXDUMP_ERR(mic_calc, 16, "computed");
		return -1;
	}
	LOG_INF("M3: MIC verified OK");

	/* Update replay counter */
	memcpy(s->replay_counter, key->replay_counter, WPA_REPLAY_CTR_LEN);

	/* Decrypt key data if encrypted */
	uint8_t decrypted[256];
	const uint8_t *kde_data;
	size_t kde_len;

	if (key_info & WPA_KEY_INFO_ENCR) {
		if (key_data_len < 8 || (key_data_len % 8) != 0) {
			LOG_ERR("M3: invalid encrypted key data length: %u", key_data_len);
			return -1;
		}
		int n = (key_data_len / 8) - 1;

		if ((size_t)(n * 8) > sizeof(decrypted)) {
			LOG_ERR("M3: key data too large");
			return -1;
		}

		if (aes_unwrap(kek, KEK_LEN, n, key_data, decrypted) != 0) {
			LOG_ERR("M3: AES unwrap failed");
			return -1;
		}
		kde_data = decrypted;
		kde_len = n * 8;
		LOG_INF("M3: key data decrypted (%d bytes)", (int)kde_len);
	} else {
		kde_data = key_data;
		kde_len = key_data_len;
	}

	/* Extract GTK from key data (before sending M4) */
	size_t gtk_len = 0;
	uint8_t gtk_idx = 0;
	const uint8_t *gtk = wpa_find_gtk_kde(kde_data, kde_len, &gtk_len, &gtk_idx);

	/* Save keys and M4 params for deferred send+install.
	 * We CANNOT send M4 from within the firmware's EAPOL RX callback —
	 * the firmware thread is blocked waiting for us to return.
	 * Defer everything to the system work queue. */
	memcpy(pending_keys.tk, tk, TK_LEN);
	pending_keys.vif_idx = s->vif_idx;
	pending_keys.sta_idx = s->sta_idx;

	if (gtk && gtk_len >= TK_LEN) {
		memcpy(pending_keys.gtk, gtk, TK_LEN);
		memcpy(pending_keys.gtk_rsc, key->key_rsc, 8);
		pending_keys.gtk_idx = gtk_idx;
		pending_keys.has_gtk = true;
	} else {
		pending_keys.has_gtk = false;
		LOG_WRN("M3: no GTK found in key data");
	}

	s->state = WPA_STATE_COMPLETED;

	/* Defer M4 send + key install to system work queue.
	 * Cannot TX from firmware's EAPOL RX callback context. */
	LOG_INF("M3 processed, scheduling M4+keys");
	k_work_schedule(&install_keys_dwork, K_MSEC(10));

	return 0;
}

/*
 * Process Group Key Handshake message 1 (AP → STA).
 *
 * The AP sends a new GTK encrypted with our KEK.  We must:
 *  1. Verify MIC using KCK
 *  2. Decrypt key data using KEK (AES unwrap)
 *  3. Extract and install the new GTK
 *  4. Send Group Key Handshake message 2 (acknowledgement)
 */
static int wpa_process_group_1(struct wpa_sm *s, const struct wpa_eapol_key *key,
			       const uint8_t *eapol_frame, size_t eapol_len)
{
	uint16_t key_info = be16(key->key_info);
	uint16_t key_data_len = be16(key->key_data_length);
	const uint8_t *key_data = (const uint8_t *)key + EAPOL_KEY_HDR_LEN;
	const uint8_t *kck = s->ptk;
	const uint8_t *kek = s->ptk + KCK_LEN;

	LOG_INF("GTK rekey: key_info=0x%04x data_len=%u", key_info, key_data_len);

	/* Verify MIC */
	uint8_t mic_calc[16];
	uint8_t mic_save[16];
	uint8_t eapol_copy[512];

	if (eapol_len > sizeof(eapol_copy)) {
		LOG_ERR("GTK rekey: frame too large");
		return -1;
	}
	memcpy(eapol_copy, eapol_frame, eapol_len);

	struct wpa_eapol_key *key_copy =
		(struct wpa_eapol_key *)(eapol_copy + sizeof(struct ieee802_1x_hdr));
	memcpy(mic_save, key_copy->key_mic, 16);
	memset(key_copy->key_mic, 0, 16);

	if (wpa_eapol_key_mic(kck, eapol_copy, eapol_len, mic_calc) != 0) {
		LOG_ERR("GTK rekey: MIC computation failed");
		return -1;
	}
	if (memcmp(mic_save, mic_calc, 16) != 0) {
		LOG_ERR("GTK rekey: MIC verification FAILED");
		return -1;
	}

	/* Update replay counter */
	memcpy(s->replay_counter, key->replay_counter, WPA_REPLAY_CTR_LEN);

	/* Decrypt key data */
	uint8_t decrypted[256];
	const uint8_t *kde_data;
	size_t kde_len;

	if (key_info & WPA_KEY_INFO_ENCR) {
		if (key_data_len < 8 || (key_data_len % 8) != 0) {
			LOG_ERR("GTK rekey: invalid key data length: %u", key_data_len);
			return -1;
		}
		int n = (key_data_len / 8) - 1;

		if ((size_t)(n * 8) > sizeof(decrypted)) {
			LOG_ERR("GTK rekey: key data too large");
			return -1;
		}
		if (aes_unwrap(kek, KEK_LEN, n, key_data, decrypted) != 0) {
			LOG_ERR("GTK rekey: AES unwrap failed");
			return -1;
		}
		kde_data = decrypted;
		kde_len = n * 8;
	} else {
		kde_data = key_data;
		kde_len = key_data_len;
	}

	/* Extract GTK */
	size_t gtk_len = 0;
	uint8_t gtk_idx = 0;
	const uint8_t *gtk = wpa_find_gtk_kde(kde_data, kde_len, &gtk_len, &gtk_idx);

	if (!gtk || gtk_len < TK_LEN) {
		LOG_ERR("GTK rekey: no GTK in key data");
		return -1;
	}

	/* Install new GTK immediately — we're already encrypted */
	LOG_INF("Installing new GTK (idx=%u)", gtk_idx);
	int ret = bl_wifi_set_sta_key_internal(s->vif_idx, s->sta_idx, WIFI_WPA_ALG_CCMP, gtk_idx,
					       0, (uint8_t *)key->key_rsc, 6, (uint8_t *)gtk,
					       TK_LEN, false);
	if (ret != 0) {
		LOG_WRN("GTK install failed: %d", ret);
	}

	/* Send Group Key Handshake message 2 */
	uint8_t snonce_save[WPA_NONCE_LEN];

	memcpy(snonce_save, s->snonce, WPA_NONCE_LEN);
	memset(s->snonce, 0, WPA_NONCE_LEN);

	uint16_t ki = WPA_KEY_INFO_TYPE_AES_SHA | WPA_KEY_INFO_MIC | WPA_KEY_INFO_SECURE;

	ret = wpa_send_eapol(s, ki, NULL, 0);

	memcpy(s->snonce, snonce_save, WPA_NONCE_LEN);

	if (ret != 0) {
		LOG_ERR("GTK rekey: response send failed");
	} else {
		LOG_INF("GTK rekey complete");
	}

	return ret;
}

/* -------- Public API -------- */

/*
 * Reset the WPA SM — clear all crypto material and state.
 * Called on disconnect to ensure clean state for next connection.
 */
void wpa_sm_reset(void)
{
	sm.state = WPA_STATE_IDLE;
	memset(sm.pmk, 0, sizeof(sm.pmk));
	memset(sm.ptk, 0, sizeof(sm.ptk));
	sm.ptk_set = false;
	memset(&pending_keys, 0, sizeof(pending_keys));
}

/*
 * Initialize the WPA SM with connection parameters.
 * Called from wpa_sta_connect callback.
 */
void wpa_sm_init(const wifi_connect_parm_t *p)
{
	memset(&sm, 0, sizeof(sm));
	sm.state = WPA_STATE_WAIT_M1;
	sm.vif_idx = p->vif_idx;
	sm.sta_idx = p->sta_idx;
	memcpy(sm.own_addr, p->mac, ETH_ALEN);
	memcpy(sm.bssid, p->bssid, ETH_ALEN);

	/* Firmware may not populate mac/bssid — get from internal APIs */
	static const uint8_t zero_addr[ETH_ALEN] = {0};

	if (memcmp(sm.own_addr, zero_addr, ETH_ALEN) == 0) {
		hwinfo_get_device_id(sm.own_addr, ETH_ALEN);
	}
	if (memcmp(sm.bssid, zero_addr, ETH_ALEN) == 0) {
		bl_wifi_get_assoc_bssid_internal(sm.vif_idx, sm.bssid);
	}

	LOG_INF("Deriving PMK: ssid_len=%d pass_len=%zu", p->ssid.len, strlen(p->passphrase));

	if (pbkdf2_sha1(p->passphrase, p->ssid.ssid, p->ssid.len, sm.pmk) != 0) {
		LOG_ERR("PBKDF2 failed");
		sm.state = WPA_STATE_IDLE;
		return;
	}

	LOG_INF("PMK: %02x%02x%02x%02x %02x%02x%02x%02x", sm.pmk[0], sm.pmk[1], sm.pmk[2],
		sm.pmk[3], sm.pmk[4], sm.pmk[5], sm.pmk[6], sm.pmk[7]);
	LOG_INF("WPA SM ready: vif=%u sta=%u "
		"own=%02x:%02x:%02x:%02x:%02x:%02x "
		"bssid=%02x:%02x:%02x:%02x:%02x:%02x",
		sm.vif_idx, sm.sta_idx, sm.own_addr[0], sm.own_addr[1], sm.own_addr[2],
		sm.own_addr[3], sm.own_addr[4], sm.own_addr[5], sm.bssid[0], sm.bssid[1],
		sm.bssid[2], sm.bssid[3], sm.bssid[4], sm.bssid[5]);
}

/*
 * Process received EAPOL frame.
 * Called from firmware via wpa_sta_rx_eapol callback.
 *
 * @src: source MAC address
 * @buf: EAPOL frame (starts at ieee802_1x_hdr, no Ethernet header)
 * @len: frame length
 */
int wpa_sm_rx_eapol(uint8_t *src, uint8_t *buf, uint32_t len)
{
	struct ieee802_1x_hdr *hdr;
	struct wpa_eapol_key *key;
	uint16_t key_info;
	uint16_t key_data_len;

	if (len < sizeof(struct ieee802_1x_hdr) + EAPOL_KEY_HDR_LEN) {
		LOG_WRN("EAPOL frame too short: %u", len);
		return -1;
	}

	/* Update BSSID from EAPOL source if not set (firmware may not
	 * populate bssid in wifi_connect_parm_t before wpa_sta_connect) */
	static const uint8_t zero_addr[ETH_ALEN] = {0};

	if (memcmp(sm.bssid, zero_addr, ETH_ALEN) == 0 && src) {
		memcpy(sm.bssid, src, ETH_ALEN);
		LOG_INF("BSSID set from EAPOL src: %02x:%02x:%02x:%02x:%02x:%02x", sm.bssid[0],
			sm.bssid[1], sm.bssid[2], sm.bssid[3], sm.bssid[4], sm.bssid[5]);
	}

	hdr = (struct ieee802_1x_hdr *)buf;
	if (hdr->type != IEEE802_1X_TYPE_KEY) {
		LOG_WRN("Not EAPOL-Key: type=%u", hdr->type);
		return -1;
	}

	key = (struct wpa_eapol_key *)(buf + sizeof(struct ieee802_1x_hdr));
	if (key->type != EAPOL_KEY_TYPE_RSN) {
		LOG_WRN("Not RSN key type: %u", key->type);
		return -1;
	}

	key_info = be16(key->key_info);
	key_data_len = be16(key->key_data_length);

	LOG_INF("RX EAPOL-Key: info=0x%04x data_len=%u state=%d", key_info, key_data_len, sm.state);
	LOG_HEXDUMP_DBG(buf, len > 64 ? 64 : len, "RX EAPOL");

	/* Verify descriptor version */
	if ((key_info & WPA_KEY_INFO_TYPE_MASK) != WPA_KEY_INFO_TYPE_AES_SHA) {
		LOG_ERR("Unsupported key descriptor version: %u",
			key_info & WPA_KEY_INFO_TYPE_MASK);
		return -1;
	}

	/* Must be from AP (ACK bit set) */
	if (!(key_info & WPA_KEY_INFO_ACK)) {
		LOG_WRN("Not an AP message: info=0x%04x", key_info);
		return -1;
	}

	/* Group Key Handshake: ACK set, KEY_TYPE (pairwise) clear */
	if (!(key_info & WPA_KEY_INFO_KEY_TYPE)) {
		if (sm.state != WPA_STATE_COMPLETED) {
			LOG_WRN("GTK rekey before 4-way complete (state=%d)", sm.state);
			return -1;
		}
		return wpa_process_group_1(&sm, key, buf, len);
	}

	/* 4-way handshake: pairwise + ACK */
	if (!(key_info & WPA_KEY_INFO_MIC)) {
		/* No MIC = Message 1 */
		if (sm.state != WPA_STATE_WAIT_M1) {
			LOG_WRN("M1 received in unexpected state %d", sm.state);
			/* Reset and process anyway */
			sm.state = WPA_STATE_WAIT_M1;
		}
		return wpa_process_m1(&sm, key, key_data_len);
	} else {
		/* MIC set = Message 3 */
		if (sm.state == WPA_STATE_COMPLETED) {
			/* AP retransmitted M3 after keys installed.
			 * Ignore — M4 was sent, AP will accept it shortly. */
			LOG_INF("M3 retransmit ignored (keys already installed)");
			return 0;
		}
		if (sm.state != WPA_STATE_WAIT_M3) {
			LOG_WRN("M3 received in unexpected state %d", sm.state);
			return -1;
		}
		return wpa_process_m3(&sm, key, buf, len);
	}
}
