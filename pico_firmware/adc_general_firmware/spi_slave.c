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
#define PAYLOAD_LEN (4u + 4u * (unsigned)N_CH)
#define FRAME_MAX_BYTES ( \
    ( ( ( ((PAYLOAD_LEN + 4u) * 8u * 6u + 4u) / 5u ) + 16u ) + 7u ) / 8u \
)

uint8_t payload_buf[PAYLOAD_LEN];
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

static inline void pack_u32_le(uint8_t* dst, uint32_t x) {
    dst[0] = (uint8_t)(x & 0xFF);
    dst[1] = (uint8_t)((x >> 8) & 0xFF);
    dst[2] = (uint8_t)((x >> 16) & 0xFF);
    dst[3] = (uint8_t)((x >> 24) & 0xFF);
}

static inline void pack_f32_le(uint8_t* dst, float f) {
    memcpy(dst, &f, 4);
}

uint32_t crc32_ieee(const uint8_t *data, size_t length) {
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

static inline bool push_bit(uint8_t* out, uint32_t* bitpos, uint8_t bit) {
    uint32_t byte_i = (*bitpos) >> 3;
    uint32_t bit_i  = (*bitpos) & 7;
    if (byte_i >= FRAME_MAX_BYTES) return false;
    if (bit) out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
    (*bitpos)++;
    return true;
}

static bool bit_stuff(const uint8_t* in, size_t len,
                      uint8_t* out, uint32_t* bitpos) {
    int ones = 0;

    for (size_t i = 0; i < len; i++) {
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (in[i] >> b) & 1u;
            if (!push_bit(out, bitpos, bit)) return false;

            if (bit) ones++;
            else     ones = 0;

            if (ones == 5) {
                if (!push_bit(out, bitpos, 0)) return false;
                ones = 0;
            }
        }
    }
    return true;
}

static inline float adc_counts_to_volts(uint16_t raw) {

#if ADC_DEADZONE > 0 // I just love the C preprocessor
    if (raw < ADC_DEADZONE)
        return 0.0f;
    else if (raw > (uint16_t)(ADC_COUNTS_MAX) - ADC_DEADZONE)
        return ADC_VREF;
#endif

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
    pack_u32_le(&payload[0], now_us);

    for (int i = 0; i < N_CH; i++) {
        float f = read_channel(i);
        size_t off = 4u + 4u * (size_t)i;
        pack_f32_le(&payload[off], f);
    }
}

// START + STUFF(payload|crc) + END
static uint32_t build_frame(uint8_t *out, const uint8_t *payload) {
    // tmp = payload || crc_le
    uint8_t tmp[PAYLOAD_LEN + 4u];
    memcpy(tmp, payload, PAYLOAD_LEN);

    uint32_t crc = crc32_ieee(payload, PAYLOAD_LEN);
    pack_u32_le(&tmp[PAYLOAD_LEN], crc);

    memset(out, 0, FRAME_MAX_BYTES);
    out[0] = FLAG_BYTE;

    uint32_t bitpos = 8; // after first flag byte
    if (!bit_stuff(tmp, sizeof(tmp), out, &bitpos)) return 0;

    // Basically take the ceiling of bitpos/8 and add one more byte for the trailing flag
    uint32_t bytes = (bitpos + 7u) >> 3;
    if (bytes + 1u > FRAME_MAX_BYTES) return 0;

    // Put trailing flag at the next byte boundary
    out[bytes] = FLAG_BYTE;
    return bytes + 1u;
}

static void answer_SPI(uint8_t* payload, uint8_t* frame_buf) {
    if (data_chan >= 0) dma_channel_abort(data_chan);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    spi_get_hw(SPI_PORT)->icr = SPI_SSPICR_RORIC_BITS;
    while (spi_is_readable(SPI_PORT))
        (void)spi_get_hw(SPI_PORT)->dr;

    uint32_t now_us = (uint32_t)time_us_64();

    build_payload(payload_buf, now_us);
    uint32_t frame_len = build_frame(frame_buf, payload_buf);
    if (frame_len == 0) {
        set_gpio_hi_z(PIN_TX); // to avoid sending garbage if frame building fails
        return;
    }

    dma_channel_set_read_addr(data_chan, frame_buf, false);
    dma_channel_set_trans_count(data_chan, (uint32_t)frame_len, true);
}

void irq_handler(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
        answer_SPI(payload_buf, frame_buf);
    } else if (events & GPIO_IRQ_EDGE_RISE) {
        if (data_chan >= 0) { dma_channel_abort(data_chan); }
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
