// 1 = fake, 0 = real ADC
#define USE_FAKE_DATA   0

// Number of channels (RP2040 supports max 4)
#define N_CH            2

// ADC GPIO pins (must be GPIO 26–29)
#define ADC_GPIOS       {26, 27}

// Linear conversion: value = m * volts + b
#define CONV_M          {1.0f, 1.0f}
#define CONV_B          {0.0f, 0.0f}

// ADC parameters
#define ADC_DEADZONE    0 // counts near 0 or max that we consider to be exactly 0V or VREF to avoid noise
#define ADC_VREF        3.3f // max voltage on ADC input
#define ADC_COUNTS_MAX  4095.0f // rp2040 12-bit ADC max count

// SPI pins and LED
#define SPI_PORT spi0
#define PIN_RX   4
#define PIN_CS   9
#define PIN_SCK  6
#define PIN_TX   7
#define LED 25