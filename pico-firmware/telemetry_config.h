/* =========================
   TELEMETRY CONFIGURATION
   ========================= */

// 1 = fake, 0 = real ADC
#define USE_FAKE_DATA   1   

// Number of channels (RP2040 supports max 4)
#define N_CH            2

// ADC GPIO pins (must be GPIO 26–29)
#define ADC_GPIOS       { 26, 27 }

// Linear conversion: value = m * volts + b
#define CONV_M          { 1.0f, 1.0f }
#define CONV_B          { 0.0f, 0.0f }

// ADC parameters (probably should leave alone)
#define ADC_VREF        3.3f
#define ADC_COUNTS_MAX  4095.0f

/* =========================
   END CONFIG
   ========================= */
