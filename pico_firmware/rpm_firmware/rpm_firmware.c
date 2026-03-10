#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/sync.h"

#include "../framed_spi.h"

#define SPI_PORT spi0
#define PIN_RX   4
#define PIN_CS   9
#define PIN_SCK  6
#define PIN_TX   7
#define LED      25

#define HALL_PIN_L 15
#define HALL_PIN_R 14
#define PPR      32

#define MIN_RPM  1.0f
#define MAX_RPM  5000.0f
#define RPM_TIMEOUT_US \
    ((uint32_t)((60.0e6f / (MIN_RPM * (float)PPR))))

#define PAYLOAD_LEN 12
#define FRAME_MAX_BYTES FRAMED_SPI_FRAME_MAX_BYTES(PAYLOAD_LEN)

static volatile uint32_t last_rise_l_us = 0;
static volatile uint32_t last_rise_r_us = 0;
static volatile float motor_l_rpm = 0.0f;
static volatile float motor_r_rpm = 0.0f;

static uint8_t payload[PAYLOAD_LEN];
static uint8_t frame_buf[FRAME_MAX_BYTES];
static framed_spi_t framed;

// Checks if RPMs have been stale for too long and sets them to 0 if so
static void check_rpms_zero(void) {
    uint32_t save = save_and_disable_interrupts();

    uint32_t now = (uint32_t)time_us_64();

    if (motor_l_rpm > 0.0f &&
        (uint32_t)(now - last_rise_l_us) > RPM_TIMEOUT_US) {
        motor_l_rpm = 0.0f;
    }

    if (motor_r_rpm > 0.0f &&
        (uint32_t)(now - last_rise_r_us) > RPM_TIMEOUT_US) {
        motor_r_rpm = 0.0f;
    }

    restore_interrupts(save);
}

// Updates the given motor RPM based on a new hall sensor rise
static void update_rpm(volatile uint32_t *last_rise_us, volatile float *motor_rpm) {
    uint32_t now = (uint32_t)time_us_64();

    if (*last_rise_us == 0) {
        *last_rise_us = now;
        *motor_rpm = 0.0f;
        return;
    }

    uint32_t period_us = now - *last_rise_us;
    *last_rise_us = now;

    float raw_rpm = 60.0e6f / ((float)period_us * (float)PPR);

    if (raw_rpm >= MIN_RPM && raw_rpm <= MAX_RPM) {
        *motor_rpm = raw_rpm;
    }
}

static void build_payload(uint8_t *payload) {
    uint32_t save = save_and_disable_interrupts();

    uint32_t ts = (uint32_t)time_us_64();
    float rpm_l = motor_l_rpm;
    float rpm_r = motor_r_rpm;

    restore_interrupts(save);

    framed_spi_pack_u32_le(&payload[0], ts);
    framed_spi_pack_f32_le(&payload[4], rpm_l);
    framed_spi_pack_f32_le(&payload[8], rpm_r);
}

static void irq_handler(uint gpio, uint32_t events) {
    switch (gpio) {
        case HALL_PIN_L:
            if (events & GPIO_IRQ_EDGE_RISE) {
                update_rpm(&last_rise_l_us, &motor_l_rpm);
            }
            break;

        case HALL_PIN_R:
            if (events & GPIO_IRQ_EDGE_RISE) {
                update_rpm(&last_rise_r_us, &motor_r_rpm);
            }
            break;

        case PIN_CS:
            // Temporarily sending on rising edges because buggy PCBs
            if (events & GPIO_IRQ_EDGE_RISE) {
                build_payload(payload);
                (void)framed_spi_send_payload(&framed, payload, PAYLOAD_LEN);
            }

            if (events & GPIO_IRQ_EDGE_FALL) {
                framed_spi_end_transaction(&framed);
            }
            break;

        default:
            break;
    }
}

static void init_all(void) {
    stdio_init_all();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    irq_set_enabled(IO_IRQ_BANK0, true);

    gpio_init(HALL_PIN_L);
    gpio_init(HALL_PIN_R);
    gpio_set_dir(HALL_PIN_L, GPIO_IN);
    gpio_set_dir(HALL_PIN_R, GPIO_IN);
    gpio_set_irq_enabled_with_callback(HALL_PIN_L, GPIO_IRQ_EDGE_RISE, true, &irq_handler);
    gpio_set_irq_enabled(HALL_PIN_R, GPIO_IRQ_EDGE_RISE, true);

    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX,  GPIO_FUNC_SPI);

    framed_spi_init(&framed, SPI_PORT, PIN_TX, DREQ_SPI0_TX, frame_buf, FRAME_MAX_BYTES);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_pull_down(PIN_CS);
    gpio_set_irq_enabled(PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
}

int main(void) {
    init_all();

    while (true) {
        check_rpms_zero();
        tight_loop_contents();
    }
}