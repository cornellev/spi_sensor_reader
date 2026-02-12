#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

#define SPI_PORT spi0
#define PIN_RX  16
#define PIN_CS  17
#define PIN_SCK 18
#define PIN_TX  19
#define LED 25

// Flag byte (bit pattern 01111110). Master scans for this in the bitstream.
#define FLAG_BYTE 0x7E

// Payload is arbitrary bytes (we'll build: 4B timestamp + 4B * N floats).
// You can increase this, but keep frame buffer sized accordingly.
#define MAX_FLOATS 8
#define MAX_PAYLOAD_BYTES  (4 + 4 * MAX_FLOATS)

// Frame buffer must hold: FLAG + stuffed(payload+crc) + FLAG.
// Worst-case stuffing expands by about 20%. Add margin.
#define FRAME_MAX_BYTES 128

static uint8_t frame_buf[FRAME_MAX_BYTES];
static int data_chan;

static inline void set_gpio_hi_z(uint pin) {
    io_bank0_hw->io[pin].ctrl =
        (io_bank0_hw->io[pin].ctrl & ~IO_BANK0_GPIO0_CTRL_OEOVER_BITS) |
        (IO_BANK0_GPIO0_CTRL_OEOVER_VALUE_DISABLE << IO_BANK0_GPIO0_CTRL_OEOVER_LSB);
}

static void configure_dma(void) {
    data_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_SPI0_TX);

    dma_channel_configure(
        data_chan,
        &c,
        &spi_get_hw(SPI_PORT)->dr,
        frame_buf,
        1,
        false
    );
}

// IEEE CRC32 
static uint32_t calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

// Writes bits into frame_buf sequentially; bitpos 0 maps to bit 7 of byte 0.
static bool put_bit(uint8_t *out, size_t out_cap_bytes, int *bitpos, uint8_t bit) {
    int bp = *bitpos;
    int byte_i = bp >> 3;
    int bit_i  = bp & 7;
    if ((size_t)byte_i >= out_cap_bytes) return false;

    if (bit) out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
    *bitpos = bp + 1;
    return true;
}

static bool put_byte_msb(uint8_t *out, size_t cap, int *bitpos, uint8_t v) {
    for (int b = 7; b >= 0; b--) {
        if (!put_bit(out, cap, bitpos, (v >> b) & 1u)) return false;
    }
    return true;
}

static bool put_bytes_stuffed(uint8_t *out, size_t cap, int *bitpos,
                              const uint8_t *data, size_t nbytes,
                              int *ones)
{
    for (size_t i = 0; i < nbytes; i++) {
        uint8_t v = data[i];
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (v >> b) & 1u;

            if (!put_bit(out, cap, bitpos, bit)) return false;

            if (bit) {
                (*ones)++;
                if (*ones == 5) {
                    if (!put_bit(out, cap, bitpos, 0)) return false;
                    *ones = 0;
                }
            } else {
                *ones = 0;
            }
        }
    }
    return true;
}

static inline int bytes_used_from_bitpos(int bitpos) {
    return (bitpos + 7) >> 3;
}

// Fake data generation (sine)
static inline float fake_signal_from_timestamp(uint32_t now_us) {
    float t = (float)now_us * 1e-6f;
    return 127.0f * sinf(t) + 127.0f;
}

static size_t build_payload(uint8_t *payload, size_t payload_cap, uint32_t now_us, int n_floats) {
    if (n_floats < 0) n_floats = 0;
    if (n_floats > MAX_FLOATS) n_floats = MAX_FLOATS;

    size_t need = 4 + (size_t)(4 * n_floats);
    if (need > payload_cap) return 0;

    // LITTLE ENDIAN timestamp 
    payload[0] = (uint8_t)(now_us >> 0);
    payload[1] = (uint8_t)(now_us >> 8);
    payload[2] = (uint8_t)(now_us >> 16);
    payload[3] = (uint8_t)(now_us >> 24);

    // TODO:
    // Read real sensor data here

    for (int i = 0; i < n_floats; i++) {
        union { float f; uint32_t u; } x = { .f = fake_signal_from_timestamp(now_us) };
        size_t off = 4 + (size_t)(4 * i);
        payload[off + 0] = (uint8_t)(x.u >> 0);
        payload[off + 1] = (uint8_t)(x.u >> 8);
        payload[off + 2] = (uint8_t)(x.u >> 16);
        payload[off + 3] = (uint8_t)(x.u >> 24);
    }

    return need;
}

// START + STUFF(payload|crc) + END
static int build_frame(uint8_t *out, size_t out_cap,
                       const uint8_t *payload, size_t payload_len) {
    memset(out, 0, out_cap);
    int bitpos = 0;

    // Start flag
    if (!put_byte_msb(out, out_cap, &bitpos, FLAG_BYTE)) return -1;

    // Compute CRC
    uint32_t crc = calculate_crc32(payload, payload_len);
    uint8_t crc_le[4] = {
        (uint8_t)(crc >> 0),
        (uint8_t)(crc >> 8),
        (uint8_t)(crc >> 16),
        (uint8_t)(crc >> 24),
    };

    int ones = 0; 

    if (!put_bytes_stuffed(out, out_cap, &bitpos, payload, payload_len, &ones)) return -1;
    if (!put_bytes_stuffed(out, out_cap, &bitpos, crc_le, sizeof(crc_le), &ones)) return -1;

    // End flag
    if (!put_byte_msb(out, out_cap, &bitpos, FLAG_BYTE)) return -1;

    return bytes_used_from_bitpos(bitpos);
}


void irq_handler(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL) {
        gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

        spi_get_hw(SPI_PORT)->icr = SPI_SSPICR_RORIC_BITS;
        while (spi_is_readable(SPI_PORT)) (void)spi_get_hw(SPI_PORT)->dr;

        uint32_t now_us = (uint32_t)time_us_64();

        uint8_t payload[MAX_PAYLOAD_BYTES];
        const int n_floats = 2;
        size_t payload_len = build_payload(payload, sizeof(payload), now_us, n_floats);
        if (payload_len == 0) return;

        int frame_len = build_frame(frame_buf, sizeof(frame_buf), payload, payload_len);
        if (frame_len <= 0) return;

        dma_hw->ch[data_chan].read_addr = (uintptr_t)frame_buf;
        dma_channel_set_trans_count(data_chan, (uint32_t)frame_len, false);
        dma_start_channel_mask(1u << data_chan);
    }
    else if (events & GPIO_IRQ_EDGE_RISE) {
        set_gpio_hi_z(PIN_TX);
    }
}

int main()
{
    stdio_init_all();
    configure_dma();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_slave(SPI_PORT, true);

    gpio_set_function(PIN_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX,  GPIO_FUNC_SPI);

    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, false);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_CS,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &irq_handler);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
