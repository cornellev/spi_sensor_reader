#include "framed_spi.h"

#include <string.h>

#include "hardware/gpio.h"

static uint32_t crc32_ieee(const uint8_t *data, size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)data[i];
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static inline bool push_bit(uint8_t *out,
                            uint32_t out_cap,
                            uint32_t *bitpos,
                            uint8_t bit) {
    uint32_t byte_i = (*bitpos) >> 3;
    uint32_t bit_i  = (*bitpos) & 7u;

    if (byte_i >= out_cap) return false;
    if (bit) out[byte_i] |= (uint8_t)(1u << (7u - bit_i));
    (*bitpos)++;
    return true;
}

static bool bit_stuff(const uint8_t *in,
                      size_t in_len,
                      uint8_t *out,
                      uint32_t out_cap,
                      uint32_t *bitpos) {
    int ones = 0;

    for (size_t i = 0; i < in_len; i++) {
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (in[i] >> b) & 1u;
            if (!push_bit(out, out_cap, bitpos, bit)) return false;

            if (bit) ones++;
            else ones = 0;

            if (ones == 5) {
                if (!push_bit(out, out_cap, bitpos, 0)) return false;
                ones = 0;
            }
        }
    }
    return true;
}

static uint32_t build_frame(const uint8_t *payload,
                            uint32_t payload_len,
                            uint8_t *out,
                            uint32_t out_cap) {
    uint8_t tmp[payload_len + 4u];

    memcpy(tmp, payload, payload_len);
    uint32_t crc = crc32_ieee(payload, payload_len);
    framed_spi_pack_u32_le(&tmp[payload_len], crc);

    memset(out, 0, out_cap);
    out[0] = FRAMED_SPI_FLAG_BYTE;

    uint32_t bitpos = 8u;
    if (!bit_stuff(tmp, payload_len + 4u, out, out_cap, &bitpos)) return 0;

    uint32_t bytes = (bitpos + 7u) >> 3;
    if (bytes + 1u > out_cap) return 0;

    out[bytes] = FRAMED_SPI_FLAG_BYTE;
    return bytes + 1u;
}

void framed_spi_pack_u32_le(uint8_t *dst, uint32_t x) {
    dst[0] = (uint8_t)(x & 0xFFu);
    dst[1] = (uint8_t)((x >> 8) & 0xFFu);
    dst[2] = (uint8_t)((x >> 16) & 0xFFu);
    dst[3] = (uint8_t)((x >> 24) & 0xFFu);
}

void framed_spi_pack_f32_le(uint8_t *dst, float f) {
    memcpy(dst, &f, 4);
}

void framed_spi_set_tx_hiz(const framed_spi_t *ctx) {
    gpio_set_function(ctx->pin_tx, GPIO_FUNC_SIO);
    gpio_set_dir(ctx->pin_tx, GPIO_IN);
    gpio_disable_pulls(ctx->pin_tx);
}

void framed_spi_init(framed_spi_t *ctx,
                     spi_inst_t *spi,
                     uint pin_tx,
                     uint dreq_tx,
                     uint8_t *frame_buf,
                     uint32_t frame_buf_cap) {
    ctx->spi = spi;
    ctx->pin_tx = pin_tx;
    ctx->dreq_tx = dreq_tx;
    ctx->frame_buf = frame_buf;
    ctx->frame_buf_cap = frame_buf_cap;
    ctx->frame_len = 0;
    ctx->dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(ctx->dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, dreq_tx);

    dma_channel_configure(ctx->dma_chan,
                          &c,
                          &spi_get_hw(spi)->dr,
                          frame_buf,
                          0,
                          false);

    framed_spi_set_tx_hiz(ctx);
}

void framed_spi_deinit(framed_spi_t *ctx) {
    if (ctx->dma_chan >= 0) {
        dma_channel_unclaim(ctx->dma_chan);
        ctx->dma_chan = -1;
    }
}

void framed_spi_abort(framed_spi_t *ctx) {
    if (ctx->dma_chan >= 0) {
        dma_channel_abort(ctx->dma_chan);
    }
}

uint32_t framed_spi_send_payload(framed_spi_t *ctx,
                                 const uint8_t *payload,
                                 uint32_t payload_len) {
    framed_spi_abort(ctx);
    gpio_set_function(ctx->pin_tx, GPIO_FUNC_SPI);

    // Clear overrun and drain RX FIFO.
    spi_get_hw(ctx->spi)->icr = SPI_SSPICR_RORIC_BITS;
    while (spi_is_readable(ctx->spi)) {
        (void)spi_get_hw(ctx->spi)->dr;
    }

    uint32_t len = build_frame(payload,
                               payload_len,
                               ctx->frame_buf,
                               ctx->frame_buf_cap);
    if (len == 0) {
        framed_spi_set_tx_hiz(ctx);
        return 0;
    }

    ctx->frame_len = len;
    dma_channel_set_read_addr(ctx->dma_chan, ctx->frame_buf, false);
    dma_channel_set_trans_count(ctx->dma_chan, len, true);
    return len;
}

void framed_spi_end_transaction(framed_spi_t *ctx) {
    framed_spi_abort(ctx);
    framed_spi_set_tx_hiz(ctx);
}