#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

#include "../framed_spi.h"

// ====================================================================
//
//     ↓ telemetry code ↓
//
// ====================================================================

#define N_CH 2
#define LED 25

#define SPI_PORT spi0
#define PIN_RX   4
#define PIN_CS   9
#define PIN_SCK  6
#define PIN_TX   7

#define PAYLOAD_LEN (4u + 4u * (unsigned)N_CH)
#define FRAME_MAX_BYTES FRAMED_SPI_FRAME_MAX_BYTES(PAYLOAD_LEN)

typedef struct {
    uint16_t raw_shunt;
    uint16_t raw_bus;
    uint16_t raw_current;
    uint16_t raw_power;
} ina226_data_t;

static ina226_data_t sensor_data;

static uint8_t payload_buf[PAYLOAD_LEN];
static uint8_t frame_buf[FRAME_MAX_BYTES];
static framed_spi_t framed;

static void build_payload(uint8_t *payload, uint32_t now_us) {
    framed_spi_pack_u32_le(&payload[0], now_us);

    int16_t signed_shunt = (int16_t)sensor_data.raw_shunt;
    float shunt_mv = signed_shunt * (2.5f / 1000.0f) * (32.0f / 27.0f);

    framed_spi_pack_f32_le(&payload[4], shunt_mv);     // Current
    framed_spi_pack_f32_le(&payload[8], 0.0f);         // Voltage
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

// ====================================================================
//
//     ↓ power monitor i2c read code ↓
//
// ====================================================================

// INA226 I2C Configuration
#define I2C_PORT i2c1
#define SDA_PIN 14
#define SCL_PIN 15
#define I2C_FREQ 400000

// INA226 Default I2C Address
#define INA226_ADDRESS 0x45 // A1=VCC, A0=VCC

// INA226 Register Addresses
#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNT_V     0x01
#define INA226_REG_BUS_V       0x02
#define INA226_REG_POWER       0x03
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MASK_ENABLE 0x06
#define INA226_REG_ALERT_LIMIT 0x07
#define INA226_REG_MANUF_ID    0xFE
#define INA226_REG_DIE_ID      0xFF

// Configuration values
#define INA226_CONFIG_RESET                0x8000
#define INA226_CONFIG_AVG_16               0x0400
#define INA226_CONFIG_VBUSCT_1100US        0x0100
#define INA226_CONFIG_VSHCT_1100US         0x0020
#define INA226_CONFIG_MODE_SHUNT_BUS_CONT  0x0007

bool ina226_write_register(uint8_t reg, uint16_t value) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = value & 0xFF;

    int result = i2c_write_blocking(I2C_PORT, INA226_ADDRESS, buffer, 3, false);
    return result == 3;
}

bool ina226_read_register(uint8_t reg, uint16_t *value) {
    uint8_t buffer[2];

    int result = i2c_write_blocking(I2C_PORT, INA226_ADDRESS, &reg, 1, true);
    if (result != 1) return false;

    result = i2c_read_blocking(I2C_PORT, INA226_ADDRESS, buffer, 2, false);
    if (result != 2) return false;

    *value = ((uint16_t)buffer[0] << 8) | buffer[1];
    return true;
}

bool ina226_init(void) {
    uint16_t manuf_id, die_id;
    (void)ina226_read_register(INA226_REG_MANUF_ID, &manuf_id);
    (void)ina226_read_register(INA226_REG_DIE_ID, &die_id);

    (void)ina226_write_register(INA226_REG_CONFIG, INA226_CONFIG_RESET);
    sleep_ms(1);

    uint16_t config = INA226_CONFIG_AVG_16 |
                      INA226_CONFIG_VBUSCT_1100US |
                      INA226_CONFIG_VSHCT_1100US |
                      INA226_CONFIG_MODE_SHUNT_BUS_CONT;

    return ina226_write_register(INA226_REG_CONFIG, config);
}

static void initialize(void) {
    stdio_init_all();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, false);

    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    framed_spi_init(
        &framed,
        SPI_PORT,
        PIN_TX,
        DREQ_SPI0_TX,
        frame_buf,
        FRAME_MAX_BYTES
    );

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_pull_down(PIN_CS);  // active-high CS idle low
    gpio_set_irq_enabled_with_callback(
        PIN_CS,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &irq_handler
    );

    (void)ina226_init();
}

int main(void) {
    initialize();

    while (true) {
        (void)ina226_read_register(INA226_REG_SHUNT_V, &sensor_data.raw_shunt);
        // int16_t signed_shunt = (int16_t)sensor_data.raw_shunt;
        // float shunt_mv = signed_shunt * (2.5f / 1000.0f) * (32.0f / 27.0f); // Adjusted for calibration
        // printf("Shunt Voltage: %.3f mV\n", shunt_mv);
        tight_loop_contents();
    }

    return 0;
}