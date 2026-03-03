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
#define LED      25

#define HALL_PIN_L 2
#define HALL_PIN_R 3
#define PPR      32

#define MIN_RPM  1.0f
#define MAX_RPM  5000.0f
#define RPM_TIMEOUT_US \
    ((uint32_t)((60.0e6f / (MIN_RPM * (float)PPR))))

#define FLAG_BYTE 0x7E

#define PAYLOAD_LEN 12
#define CRC_LEN     4

#define FRAME_MAX_BYTES ( \
    ( ( ( ((PAYLOAD_LEN + CRC_LEN) * 8u * 6u + 4u) / 5u ) + 16u ) + 7u ) / 8u \
)

static volatile uint32_t last_rise_l_us = 0;
static volatile uint32_t last_rise_r_us = 0;
static volatile float motor_l_rpm = 0.0f;
static volatile float motor_r_rpm = 0.0f;

static int data_chan = -1;

static uint8_t payload[PAYLOAD_LEN];
static uint8_t frame_buf[FRAME_MAX_BYTES];
static volatile uint32_t frame_len = 0;

static inline void set_gpio_hi_z(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

// static inline void set_tx_spi_func(void) {
//     gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
// }

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

// Set up DMA for quick SPI TX writes
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
        0,
        false
    );
}

// Checks if RPMs have been stale for too long and sets them to 0 if so
static void check_rpms_zero() {
    uint32_t now = (uint32_t)time_us_64();

    bool is_zero_l = (motor_l_rpm > 0.0f) &&
                        ((uint32_t)(now - last_rise_l_us) > RPM_TIMEOUT_US);
    bool is_zero_r = (motor_r_rpm > 0.0f) &&
                        ((uint32_t)(now - last_rise_r_us) > RPM_TIMEOUT_US);

    if (!is_zero_l && !is_zero_r) return;

    // Typically both would be stale at the same time, so large critical section is preferred here.
    uint32_t save = save_and_disable_interrupts();
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

// Updates the given motor RPM based on a new hall sensor rise, using the last rise time to compute period
static void update_rpm(volatile uint32_t* last_rise_us, volatile float* motor_rpm) {
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

// Answers an SPI read by building a frame with the current timestamp and RPMs and starting DMA to send it
static void answer_SPI(uint8_t* payload, uint8_t* frame_buf) {
    if (data_chan >= 0) { dma_channel_abort(data_chan); }
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    // Clear SPI overrun and drain RX FIFO
    spi_get_hw(SPI_PORT)->icr = SPI_SSPICR_RORIC_BITS;
    while (spi_is_readable(SPI_PORT)) { (void)spi_get_hw(SPI_PORT)->dr; }
        
    uint32_t ts = (uint32_t)time_us_64(); // pmo

    // Save locally in case of concurrent updates while building frame)
    // uint32_t save = save_and_disable_interrupts();
    float rpm_l = motor_l_rpm;
    float rpm_r = motor_r_rpm;
    // restore_interrupts(save);

    pack_u32_le(&payload[0], ts);
    pack_f32_le(&payload[4], rpm_l);
    pack_f32_le(&payload[8], rpm_r);

    uint32_t len = build_hdlc_frame(payload, PAYLOAD_LEN, frame_buf);
    if (len == 0) {
        set_gpio_hi_z(PIN_TX); // to avoid sending garbage if frame building fails
        return;
    }

    frame_len = len;

    dma_channel_set_read_addr(data_chan, frame_buf, false);
    dma_channel_set_trans_count(data_chan, frame_len, true);
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
            if (events & GPIO_IRQ_EDGE_FALL) {
                answer_SPI(payload, frame_buf, &motor_l_rpm, &motor_r_rpm);

            } else if (events & GPIO_IRQ_EDGE_RISE) {
                if (data_chan >= 0) { dma_channel_abort(data_chan); }
                set_gpio_hi_z(PIN_TX);
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

    spi_init(SPI_PORT, 1 * 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
    set_gpio_hi_z(PIN_TX);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_IN);
    gpio_pull_up(PIN_CS);
    gpio_set_irq_enabled( PIN_CS, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

    // gpio_init(PIN_TX);
    // set_gpio_hi_z(PIN_TX);
    // gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    configure_dma();
}

int main(void) {
    init_all();
    while (true) {
        // printf("%f, %f\n", motor_l_rpm, motor_r_rpm);
        check_rpms_zero();
        tight_loop_contents();
    }
}