/* CRC32 stream API for firmware blob — wraps Zephyr's crc32_ieee */
#include <zephyr/sys/crc.h>
#include "utils_crc.h"

void utils_crc32_stream_init(struct crc32_stream_ctx *ctx)
{
	ctx->crc = 0xffffffff;
}

void utils_crc32_stream_feed(struct crc32_stream_ctx *ctx, uint8_t data)
{
	ctx->crc = crc32_ieee_update(ctx->crc, &data, 1);
}

void utils_crc32_stream_feed_block(struct crc32_stream_ctx *ctx, uint8_t *data, uint32_t len)
{
	ctx->crc = crc32_ieee_update(ctx->crc, data, len);
}

uint32_t utils_crc32_stream_results(struct crc32_stream_ctx *ctx)
{
	return ctx->crc ^ 0xffffffff;
}
