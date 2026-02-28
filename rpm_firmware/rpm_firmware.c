#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/structs/io_bank0.h"

#define SPI_PORT spi0
#define PIN_RX   16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_TX   19

#define HALL_PIN 2
#define PPR      32

#define MIN_RPM  10.0f
#define MAX_RPM  8000.0f

#define FLAG_BYTE 0x7E

#define PAYLOAD_LEN 12
#define CRC_LEN     4

#define FRAME_MAX_BYTES ( \
    ( ( ( ((PAYLOAD_LEN + CRC_LEN) * 8u * 6u + 4u) / 5u ) + 16u ) + 7u ) / 8u \
)

static volatile uint32_t last_rise_us = 0;
static volatile float motor_rpm = 0.0f;

static int dma_chan = -1;

static uint8_t payload[PAYLOAD_LEN];
static uint8_t frame_buf[FRAME_MAX_BYTES];
static volatile uint32_t frame_len = 0;

static inline void set_gpio_hi_z(uint pin) {
    io_bank0_hw->io[pin].ctrl =
        (io_bank0_hw->io[pin].ctrl & ~IO_BANK0_GPIO0_CTRL_OEOVER_BITS) |
        (IO_BANK0_GPIO0_CTRL_OEOVER_VALUE_DISABLE << IO_BANK0_GPIO0_CTRL_OEOVER_LSB);
}

static inline void set_tx_spi_func(void) {
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
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

static uint32_t crc32_ieee(const uint8_t* data, size_t n) {
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

static inline bool push_bit(uint8_t* out, uint32_t* bitpos, uint8_t bit) {
    uint32_t byte_i = (*bitpos) >> 3;
    uint32_t bit_i  = (*bitpos) & 7;
    if (byte_i >= FRAME_MAX_BYTES) return false;
    if (bit) out[byte_i] |= (uint8_t)(1u << (7 - bit_i));
    (*bitpos)++;
    return true;
}

static uint32_t build_hdlc_frame(const uint8_t* pl, size_t pl_len, uint8_t* out) {
    // Build tmp = payload || crc_le
    uint8_t tmp[PAYLOAD_LEN + CRC_LEN];
    memcpy(tmp, pl, pl_len);

    uint32_t crc = crc32_ieee(pl, pl_len);
    tmp[pl_len + 0] = (uint8_t)((crc >> 0)  & 0xFF);
    tmp[pl_len + 1] = (uint8_t)((crc >> 8)  & 0xFF);
    tmp[pl_len + 2] = (uint8_t)((crc >> 16) & 0xFF);
    tmp[pl_len + 3] = (uint8_t)((crc >> 24) & 0xFF);

    memset(out, 0, FRAME_MAX_BYTES);
    out[0] = FLAG_BYTE;

    uint32_t bitpos = 8; // after first flag byte
    int ones = 0;

    for (size_t i = 0; i < (pl_len + CRC_LEN); i++) {
        for (int b = 7; b >= 0; b--) {
            uint8_t bit = (tmp[i] >> b) & 1u;

            if (!push_bit(out, &bitpos, bit)) return 0;

            if (bit) ones++;
            else ones = 0;

            if (ones == 5) {
                if (!push_bit(out, &bitpos, 0)) return 0; // stuffed 0
                ones = 0;
            }
        }
    }

    uint32_t bytes = (bitpos + 7u) >> 3;
    if (bytes + 1u > FRAME_MAX_BYTES) return 0;

    // Put trailing flag at the next byte boundary
    out[bytes] = FLAG_BYTE;
    return bytes + 1u;
}

static void configure_dma(void) {
    dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_SPI0_TX);

    dma_channel_configure(
        dma_chan,
        &c,
        &spi_get_hw(SPI_PORT)->dr,
        frame_buf,
        0,
        false
    );
}

static void irq_handler(uint gpio, uint32_t events) {
    if ((gpio == HALL_PIN) && (events & GPIO_IRQ_EDGE_RISE)) {

        uint32_t now = (uint32_t)time_us_64();

        if (last_rise_us == 0) {
            last_rise_us = now;
            motor_rpm = 0.0f;
            return;
        }

        uint32_t period_us = now - last_rise_us;
        last_rise_us = now;

        float raw_rpm = 60.0e6f / ((float)period_us * (float)PPR);

        // Validate in RPM-space only
        if (raw_rpm < MIN_RPM || raw_rpm > MAX_RPM) {
            return;
        }

        motor_rpm = raw_rpm;
        // printf("%f\n", motor_rpm);

    } else if ((gpio == PIN_CS) && (events & GPIO_IRQ_EDGE_FALL)) {

        uint32_t ts = (uint32_t)time_us_64();
        float rpm = motor_rpm;

        pack_u32_le(&payload[0], ts);
        pack_f32_le(&payload[4], rpm);
        pack_f32_le(&payload[8], rpm);

        uint32_t len = build_hdlc_frame(payload, PAYLOAD_LEN, frame_buf);
        if (len == 0) {
            set_gpio_hi_z(PIN_TX);
            return;
        }

        frame_len = len;

        set_tx_spi_func();
        dma_channel_set_read_addr(dma_chan, frame_buf, false);
        dma_channel_set_trans_count(dma_chan, frame_len, true);

    } else if ((gpio == PIN_CS) && (events & GPIO_IRQ_EDGE_RISE)) {
        if (dma_chan >= 0) dma_channel_abort(dma_chan);
        set_gpio_hi_z(PIN_TX);
    }
}

static void init_all(void) {
    stdio_init_all();

    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_init(HALL_PIN);
    gpio_set_dir(HALL_PIN, GPIO_IN);
    gpio_set_irq_enabled(
        HALL_PIN, 
        GPIO_IRQ_EDGE_RISE, 
        true
    );

    spi_init(SPI_PORT, 1 * 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_pull_up(PIN_CS);
    gpio_set_irq_enabled_with_callback(
        PIN_CS,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        &irq_handler
    );

    gpio_init(PIN_TX);
    set_gpio_hi_z(PIN_TX);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    configure_dma();
}

int main(void) {
    init_all();
    while (true) {
        // printf("%d\n", gpio_get(HALL_PIN));
        tight_loop_contents();
    }
}