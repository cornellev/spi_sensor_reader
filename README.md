# uc26_sensor_reader

HDLC-framed SPI (and GPS UART) sensor reader → POSIX shared memory (seqlock) → Python reader.

This repository provides a C++ daemon that polls multiple SPI devices at ~200 Hz,
decodes HDLC-framed payloads with CRC checking, simultaneously reads GPS over UART 
with the Waveshare SIM7600X HAT at 1Hz, and publishes the latest sensor snapshot into 
POSIX shared memory. Consumers (e.g. Python) can read the data lock-free using a sequence 
lock.

This repository also includes a minimal RP2040 SPI slave firmware in directory
(`/pico_firmware`) that reads sensor data and sends as HDLC-framed packets.

The system is designed to be extended as additional sensors are brought online.

---

## Python Data Structure

```python
# read_snapshot_dict() returns:
{
    "seq": int,                     # read_snapshot()[0]
    "global_ts": int,               # read_snapshot()[1][0]
    "power": {
        "ts": int,                  # read_snapshot()[1][1]
        "current": float,           # read_snapshot()[1][2]
        "voltage": float,           # read_snapshot()[1][3]
    },
    "steering": { 
        "ts": int,                  # read_snapshot()[1][4]
        "brake_pressure": float,    # read_snapshot()[1][5]
        "turn_angle": float,        # read_snapshot()[1][6]
    },
    "rpm_front": {
        "ts": int,                  # read_snapshot()[1][7]
        "rpm_left": float,          # read_snapshot()[1][8]
        "rpm_right": float,         # read_snapshot()[1][9]
    },
    "rpm_back": {
        "ts": int,                  # read_snapshot()[1][10]
        "rpm_left": float,          # read_snapshot()[1][11]
        "rpm_right": float,         # read_snapshot()[1][12]
    },
    "gps": {                  
        "ts": int,                  # read_snapshot()[1][13]
        "lat": float,               # read_snapshot()[1][14]
        "long": float,              # read_snapshot()[1][15]
        "heading": float,           # read_snapshot()[1][16] (usually empty)
        "speed": float,             # read_snapshot()[1][17] (m/s)
    },
    "motor": {
        "ts": int,                  # read_snapshot()[1][18]
        "rpm": float,               # read_snapshot()[1][19]
        "throttle": float,          # read_snapshot()[1][20]
    },
}
```
### Timestamp notes:
Please use sensor-specific timestamps for any sensor-level calculations (e.g., computing Δt).

The 32-bit sensor timestamps are generated independently on each sensor microcontroller.
They may not be synchronized with one another and will wrap to 0 approximately every 71 minutes.

Use the global 64-bit timestamp for bookkeeping purposes (e.g., time-series alignment and logging).
The global timestamp is applied each time the shared memory block is written.

All timestamps are in microseconds.

## Quick Start

1. **Compile & run the C++ DAQ Master / SHM Writer**
   ```bash
    g++ -O2 -std=c++17 pi_software/collect_sensors.cpp \
    -lpigpiod_if2 -lrt -pthread \
    -o collect_sensors

    sudo pigpiod
    ./collect_sensors
   ```
   Keep this running in one terminal or screen, Ctrl+C to stop cleanly. Even better
   if you register it as a systemd service. 
 
 3. **Read sensor data in Python**
    
    Use the class in your own code. See main in `read_shm.py` for reference, or
    ```python
    from read_shm import SensorShmReader
    import time
    
    reader = SensorShmReader()
    
    try:
        while True:
            snap = reader.read_snapshot() # or read_snapshot_dict(), but that's more overhead
            if snap:
                # Do whatever you want with the data
            time.sleep(0.005)  # ~200 Hz
    finally:
        reader.close()
    ```

## Electrical Section

Proposed SPI setup for UC26:

| CS GPIO | Description          | Format                                                    |
|---------|----------------------|-----------------------------------------------------------|
| 22      | Power Monitor        | u32 ts + float current + float voltage                    |
| 23      | Steering             | u32 ts +  float brake_pressure + float turn_angle         |
| 24      | Front RPM            | u32 ts + float rpm_left + float rpm_right                 |
| 25      | Back RPM             | u32 ts + float rpm_left + float rpm_right                 |
| 26      | Motor Controller     | u32 ts + float rpm + float throttle                       |

Bus: `/dev/spidev0.0`

This repository also makes it easy to flash an RP2040 with custom SPI-slave firmware that
integrates cleanly into the existing SPI polling pipeline for analog sensors. There is also firmware
for the RPM sensor which times between digital pulses. Motor controller firmware will be added soon 
which expects a combination of a digital pulse signal and analog signal. The rest of our sensors
should be analog though, so we have made it easy to configure the firmware to work with any of these
sensors.

The Pico firmware (`pico_firmware/adc_general_firmware/adc_sensor_node.c`) acts as an SPI slave device. Whenever
the master asserts chip-select, the Pico responds with a single HDLC-framed message
containing a timestamp and a fixed number of ADC readings. The payload is CRC-protected
and bit-stuffed, so it can be safely parsed by the existing HDLC decoder on the host.

To add a new Pico-based sensor, you configure its ADC channels and calibration in
`pico_firmware/adc_general_firmware/telemetry_config.h`, flash the firmware, and assign the Pico to one of the CS lines
above. From the host’s perspective, the Pico behaves like any other SPI sensor.

Suppose you want to implement the Joulemeter/Power montitor (CS GPIO 22) using a Pico with 
two analog inputs:
- Channel 0: current sense amplifier output
- Channel 1: voltage divider output

In `pico_firmware/adc_general_firmware/telemetry_config.h`:

```c
#include <stdint.h>

// 1 = fake, 0 = real ADC
#define USE_FAKE_DATA   0   

// Number of channels (RP2040 supports max 4)
#define N_CH            2

// ADC GPIO pins (must be GPIO 26–29)
constant uint8_t ADC_GPIOS[N_CH] = {26, 27};

// Linear conversion: value = m * volts + b
constant float CONV_M[N_CH] = {1.0f, 1.0f};  // I do not know the actual conversions you'll need.
constant float CONV_B[N_CH] = {0.0f, 0.0f};

// ADC parameters
#define ADC_DEADZONE    0                    // Similarly I do not know this
#define ADC_VREF        3.3f
#define ADC_COUNTS_MAX  4095.0f

// SPI pins and LED
#define SPI_PORT        spi0                 // Use what pins are available         
#define PIN_RX          4
#define PIN_CS          9
#define PIN_SCK         6
#define PIN_TX          7
#define LED             25
```

Additionally, when building the Pico firmware, ensure the `pico_firmware/adc_general_firmware` directory is added 
to the include path so `telemetry_config.h` is visible.
