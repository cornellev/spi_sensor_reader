#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "telemetry_config.h"
#include "../framed_spi.h"

// RP2040 external ADC inputs are ADC0..ADC3 on GPIO 26..29 (4 total).
#define MAX_CHANNELS 4

#if (N_CH > MAX_CHANNELS)
#error "N_CH must be <= MAX_CHANNELS (4)."
#endif
#if (N_CH < 1)
#error "N_CH must be >= 1."
#endif

#define PAYLOAD_LEN (4u + 4u * (unsigned)N_CH)
#define FRAME_MAX_BYTES FRAMED_SPI_FRAME_MAX_BYTES(PAYLOAD_LEN)

uint8_t payload_buf[PAYLOAD_LEN];
uint8_t frame_buf[FRAME_MAX_BYTES];
static framed_spi_t framed;

static inline float fake_signal_from_now(void) {
    float now_us = (float)time_us_64();
    float t = now_us * 1e-6f;
    return 127.0f * sinf(t) + 127.0f;
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
    uint8_t gpio = ADC_GPIOS[i];
    if (gpio < 26 || gpio > 29)
        return (float)NAN;

    uint input = (uint)(gpio - 26);

    adc_select_input(input);
    (void)adc_read();
    uint16_t raw = adc_read();

    float v = adc_counts_to_volts(raw);

    static const float conv_m[] = CONV_M;
    static const float conv_b[] = CONV_B;

    return conv_m[i] * v + conv_b[i];
#endif
}

static void build_payload(uint8_t *payload, uint32_t now_us) {
    framed_spi_pack_u32_le(&payload[0], now_us);

    for (int i = 0; i < N_CH; i++) {
        float f = read_channel(i);
        size_t off = 4u + 4u * (size_t)i;
        framed_spi_pack_f32_le(&payload[off], f);
    }
}

static void irq_handler(uint gpio, uint32_t events) {
    (void)gpio;

    // Temporarily sending on rising edges because buggy PCBs
    if (events & GPIO_IRQ_EDGE_RISE) {
        uint32_t now_us = (uint32_t)time_us_64();
        build_payload(payload_buf, now_us);
        (void)framed_spi_send_payload(&framed, payload_buf, PAYLOAD_LEN);
    } 
    
    if (events & GPIO_IRQ_EDGE_FALL) {
        framed_spi_end_transaction(&framed);
    }
}

int main(void) {
    stdio_init_all();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

#if !USE_FAKE_DATA
    adc_init();
    static const uint8_t adc_gpios[] = ADC_GPIOS;
    for (int i = 0; i < N_CH; i++) {
        adc_gpio_init(adc_gpios[i]);
    }
#endif

    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, false);

    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    framed_spi_init(&framed, SPI_PORT, PIN_TX, DREQ_SPI0_TX, frame_buf, FRAME_MAX_BYTES);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_set_irq_enabled_with_callback(
        PIN_CS,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &irq_handler
    );

    while (true) {
        tight_loop_contents();
    }
}