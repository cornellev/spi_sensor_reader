#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "telemetry_config.h"

#define SPI_PORT spi0
#define PIN_RX 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_TX 19
#define LED 25

// Flag byte (bit pattern 01111110). Master scans for this in the bitstream.
#define FLAG_BYTE 0x7E

// RP2040 external ADC inputs are ADC0..ADC3 on GPIO 26..29 (4 total).
#define MAX_CHANNELS 4

#if (N_CH > MAX_CHANNELS)
#error "N_CH must be <= MAX_CHANNELS (4)."
#endif
#if (N_CH < 1)
#error "N_CH must be >= 1."
#endif

// Worst-case HDLC frame size for N_CH floats:
//   payload = 4B timestamp + 4B * N_CH
//   crc     = 4B
//   stuffing worst-case: +1 bit per 5 bits
//   flags   = 2 bytes (never stuffed)
#define PAYLOAD_MAX_BYTES (4u + 4u * (unsigned)N_CH)
#define FRAME_MAX_BYTES ( \
    ( ( ( ((PAYLOAD_MAX_BYTES + 4u) * 8u * 6u + 4u) / 5u ) + 16u ) + 7u ) / 8u \
)

uint8_t payload_buf[PAYLOAD_MAX_BYTES];
uint8_t frame_buf[FRAME_MAX_BYTES];

static int data_chan = -1; // DMA channel for SPI TX 

static inline float fake_signal_from_now(void) {
    float now_us = (float)time_us_64();
    float t = now_us * 1e-6f;
    return 127.0f * sinf(t) + 127.0f;
}

// Put GPIO into high-impedance so the SPI slave releases MISO when not selected
static inline void set_gpio_hi_z(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

// Set up DMA for quick SPI TX writes
void configure_dma(void) {
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
        0, 
        false);
}

// IEEE CRC32
uint32_t calculate_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

// Writes bits into frame_buf sequentially; bitpos 0 maps to bit 7 of byte 0.
bool put_bit(uint8_t *out, size_t out_cap_bytes, int *bitpos, uint8_t bit) {
    int bp = *bitpos;
    int byte_i = bp >> 3;
    int bit_i = bp & 7;
    if ((size_t)byte_i >= out_cap_bytes)
        return false;

    if (bit)
        out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
    *bitpos = bp + 1;
    return true;
}

bool put_byte_msb(uint8_t *out, size_t cap, int *bitpos, uint8_t v) {
    for (int b = 7; b >= 0; b--) {
        if (!put_bit(out, cap, bitpos, (v >> b) & 1u))
            return false;
    }
    return true;
}

bool put_bytes_stuffed(uint8_t *out, size_t cap, int *bitpos, const uint8_t *data, size_t nbytes,
                       int *ones) {
    for (size_t i = 0; i < nbytes; i++) {
        uint8_t v = data[i];
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (v >> b) & 1u;

            if (!put_bit(out, cap, bitpos, bit))
                return false;

            if (bit) {
                (*ones)++;
                if (*ones == 5) {
                    if (!put_bit(out, cap, bitpos, 0))
                        return false;
                    *ones = 0;
                }
            } else {
                *ones = 0;
            }
        }
    }
    return true;
}

inline int bytes_used_from_bitpos(int bitpos) {
    return (bitpos + 7) >> 3;
}

static inline float adc_counts_to_volts(uint16_t raw) {
    return ((float)raw) * (ADC_VREF / ADC_COUNTS_MAX);
}

static float read_channel(int i) {
    if (i < 0 || i >= N_CH)
        return (float)NAN;

#if USE_FAKE_DATA
    (void)i;
    return fake_signal_from_now();
#else
    uint8_t gpio = adc_gpios[i];
    if (gpio < 26 || gpio > 29)
        return (float)NAN;

    // RP2040 mapping: ADC0..ADC3 correspond to GPIO 26..29
    uint input = (uint)(gpio - 26);

    adc_select_input(input);
    (void)adc_read(); // Discard first reading after switching input per
                      // datasheet recommendation
    uint16_t raw = adc_read();

    float v = adc_counts_to_volts(raw);
    return conv_m[i] * v + conv_b[i];
#endif
}

void build_payload(uint8_t *payload, uint32_t now_us) {
    // LITTLE ENDIAN timestamp
    payload[0] = (uint8_t)(now_us >> 0);
    payload[1] = (uint8_t)(now_us >> 8);
    payload[2] = (uint8_t)(now_us >> 16);
    payload[3] = (uint8_t)(now_us >> 24);

    for (int i = 0; i < N_CH; i++) {
        // OMG i get to use a union in C! WOW.
        union {
            float f;
            uint32_t u;
        } x = {.f = read_channel(i)};
        size_t off = 4 + (size_t)(4 * i);
        payload[off + 0] = (uint8_t)(x.u >> 0);
        payload[off + 1] = (uint8_t)(x.u >> 8);
        payload[off + 2] = (uint8_t)(x.u >> 16);
        payload[off + 3] = (uint8_t)(x.u >> 24);
    }
}

// START + STUFF(payload|crc) + END
int build_frame(uint8_t *out, const uint8_t *payload) {
    memset(out, 0, (size_t)FRAME_MAX_BYTES);
    int bitpos = 0;

    // Start flag
    if (!put_byte_msb(out, (size_t)FRAME_MAX_BYTES, &bitpos, FLAG_BYTE))
        return -1;

    // Compute CRC
    uint32_t crc = calculate_crc32(payload, (size_t)PAYLOAD_MAX_BYTES);
    uint8_t crc_le[4] = {
        (uint8_t)(crc >> 0),
        (uint8_t)(crc >> 8),
        (uint8_t)(crc >> 16),
        (uint8_t)(crc >> 24),
    };

    int ones = 0;

    if (!put_bytes_stuffed(out, (size_t)FRAME_MAX_BYTES, &bitpos, payload, (size_t)PAYLOAD_MAX_BYTES, &ones))
        return -1;
    if (!put_bytes_stuffed(out, (size_t)FRAME_MAX_BYTES, &bitpos, crc_le, sizeof(crc_le), &ones))
        return -1;

    // End flag
    if (!put_byte_msb(out, (size_t)FRAME_MAX_BYTES, &bitpos, FLAG_BYTE))
        return -1;

    return bytes_used_from_bitpos(bitpos);
}

void irq_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
        if (data_chan >= 0) dma_channel_abort(data_chan);
        gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

        spi_get_hw(SPI_PORT)->icr = SPI_SSPICR_RORIC_BITS;
        while (spi_is_readable(SPI_PORT))
            (void)spi_get_hw(SPI_PORT)->dr;

        uint32_t now_us = (uint32_t)time_us_64();

        build_payload(payload_buf, now_us);
        int frame_len = build_frame(frame_buf, payload_buf);
        if (frame_len <= 0)
            return;

        dma_hw->ch[data_chan].read_addr = (uintptr_t)frame_buf;
        dma_channel_set_trans_count(data_chan, (uint32_t)frame_len, false); // only transmit frame_len 
        dma_start_channel_mask(1u << data_chan);
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        set_gpio_hi_z(PIN_TX);
    }
}

int main() {
    stdio_init_all();
    configure_dma();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

#if !USE_FAKE_DATA
    adc_init();
    for (int i = 0; i < N_CH; i++) {
        // Configure GPIO for ADC 
        adc_gpio_init(adc_gpios[i]);
    }
#endif

    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_slave(SPI_PORT, true);

    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
    set_gpio_hi_z(PIN_TX);

    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, false);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true,
                                       &irq_handler);

    while (true) {
        // printf("Hello, world!\n");
        tight_loop_contents();
    }
}
