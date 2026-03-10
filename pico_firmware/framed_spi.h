#ifndef FRAMED_SPI_H
#define FRAMED_SPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Must include SDK paths when compiling with this
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "pico/types.h"

#define FRAMED_SPI_FLAG_BYTE 0x7E

// Worst-case frame size for:
//   start flag + stuffed(payload || crc32_le) + end flag
#define FRAMED_SPI_FRAME_MAX_BYTES(payload_len) \
    (((((((uint32_t)((payload_len) + 4u) * 8u * 6u + 4u) / 5u) + 16u) + 7u) / 8u))

typedef struct {
    spi_inst_t *spi;
    uint pin_tx;
    uint dreq_tx;
    int dma_chan;

    uint8_t *frame_buf;
    uint32_t frame_buf_cap;

    volatile uint32_t frame_len;
} framed_spi_t;

/** [framed_spi_pack_u32_le dst x] writes [x] to [dst] in little-endian order.
    Requires: [dst] points to at least 4 writable bytes. */
void framed_spi_pack_u32_le(uint8_t *dst, uint32_t x);

/** [framed_spi_pack_f32_le dst f] writes the IEEE-754 bytes of [f] to [dst]
    in little-endian order.
    Requires: [dst] points to at least 4 writable bytes. */
void framed_spi_pack_f32_le(uint8_t *dst, float f);

/** [framed_spi_init ctx spi pin_tx dreq_tx frame_buf frame_buf_cap] initializes
    [ctx] for framed SPI slave transmission using [spi], TX pin [pin_tx], DMA
    request [dreq_tx], and frame buffer [frame_buf].
    Requires: all pointers are non-null, [frame_buf] points to at least
    [frame_buf_cap] writable bytes, and [ctx] is not already initialized. */
void framed_spi_init(framed_spi_t *ctx,
                     spi_inst_t *spi,
                     uint pin_tx,
                     uint dreq_tx,
                     uint8_t *frame_buf,
                     uint32_t frame_buf_cap);

/** [framed_spi_deinit ctx] releases resources owned by [ctx].
    Safe to call only after [framed_spi_init]. */
void framed_spi_deinit(framed_spi_t *ctx);

/** [framed_spi_set_tx_hiz ctx] puts the TX pin in high-impedance mode so the
    slave releases MISO when inactive.
    Requires: [ctx] has been initialized. */
void framed_spi_set_tx_hiz(const framed_spi_t *ctx);

/** [framed_spi_abort ctx] aborts any in-progress TX DMA transfer for [ctx].
    Requires: [ctx] has been initialized. */
void framed_spi_abort(framed_spi_t *ctx);

/** [framed_spi_send_payload ctx payload payload_len] frames [payload] as
    FLAG + stuffed(payload || crc32_le) + FLAG, stores it in [ctx]'s frame
    buffer, and starts DMA transmission. Returns the framed byte length, or
    [0] if framing fails.
    Requires: [ctx] has been initialized and [payload] points to at least
    [payload_len] readable bytes. */
uint32_t framed_spi_send_payload(framed_spi_t *ctx,
                                 const uint8_t *payload,
                                 uint32_t payload_len);

/** [framed_spi_end_transaction ctx] aborts any active transfer and releases
    the TX pin by putting it in high-impedance mode.
    Requires: [ctx] has been initialized. */
void framed_spi_end_transaction(framed_spi_t *ctx);

#endif