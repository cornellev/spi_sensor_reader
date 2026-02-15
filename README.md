# uc26_sensor_reader

HDLC-framed SPI (and GPS UART) sensor reader → POSIX shared memory (seqlock) → Python reader.

This repository provides a C++ daemon that polls multiple SPI devices at ~200 Hz,
decodes HDLC-framed payloads with CRC checking, simultaneously reads GPS over UART 
with the Wireshare SIM7600X HAT at 1Hz, and publishes the latest sensor snapshot into 
POSIX shared memory. Consumers (e.g. Python) can read the data lock-free using a sequence 
lock.

This repository also includes a minimal RP2040 SPI slave firmware
(`pico-firmware/spi_slave.c`) that transmits reads analog sensor data and sends as
HDLC-framed packets over SPI.

The system is designed to be extended as additional sensors are brought online.

---

## Python Data Structure

```python
{
    "seq": int,               # read_snapshot()[0]
    "power": {
        "ts": int,            # read_snapshot()[1][0]
        "current": float,     # read_snapshot()[1][1]
        "voltage": float,     # read_snapshot()[1][2]
    },
    "driver": { 
        "ts": int,            # read_snapshot()[1][3]
        "throttle": float,    # read_snapshot()[1][4]
        "velocity": float,    # read_snapshot()[1][5]
        "turn_angle": float,  # read_snapshot()[1][6]
    },
    "rpm_front": {
        "ts": int,            # read_snapshot()[1][7]
        "rpm_left": float,    # read_snapshot()[1][8]
        "rpm_right": float,   # read_snapshot()[1][9]
    },
    "rpm_back": {
        "ts": int,            # read_snapshot()[1][10]
        "rpm_left": float,    # read_snapshot()[1][11]
        "rpm_right": float,   # read_snapshot()[1][12]
    },
    "gps": {                  
        "ts": int,            # read_snapshot()[1][13]
        "lat": float,         # read_snapshot()[1][14]
        "long": float,        # read_snapshot()[1][15]
    },
}
```

## Quick Start

1. **Compile & run the C++ writer**
   ```bash
    g++ -O2 -std=c++17 write_shm.cpp \
        lib/sim7x00.cpp lib/arduPi.cpp \
    -lpigpiod_if2 -lrt -pthread \
    -o shm_writer

    sudo pigpiod
    ./shm_writer
   ```
   (keep this running in one terminal — Ctrl+C to stop cleanly)
 
 2. **Read sensor data in Python**
    
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

| CS GPIO | Description          | Format                     |
|---------|----------------------|----------------------------|
| 22      | Power Monitor      | u32 ts + float current + float voltage    |
| 23      | Driver              | u32 ts + float throttle + float brake + float turn_angle  |
| 24      | Front RPM       | u32 ts + float rpm_left + float rpm_right   |
| 25      | Back RPM        | u32 ts + float rpm_left + float rpm_right   |

Bus: `/dev/spidev0.0`

This repository also makes it easy to flash an RP2040 with custom SPI-slave firmware that
integrates cleanly into the existing SPI polling pipeline.

The Pico firmware (`pico-firmware/spi_slave.c`) acts as an SPI slave device. Whenever
the master asserts chip-select, the Pico responds with a single HDLC-framed message
containing a timestamp and a fixed number of ADC readings. The payload is CRC-protected
and bit-stuffed, so it can be safely parsed by the existing HDLC decoder on the host.

To add a new Pico-based sensor, you configure its ADC channels and calibration in
`telemetry_config.h`, flash the firmware, and assign the Pico to one of the CS lines
above. From the host’s perspective, the Pico behaves like any other SPI sensor.

Suppose you want to implement the Joulemeter/Power montitor (CS GPIO 22) using a Pico with 
two analog inputs:
- Channel 0: current sense amplifier output
- Channel 1: voltage divider output

In `pico-firmware/telemetry_config.h`:

```c
// 1 = fake, 0 = real ADC
#define USE_FAKE_DATA   0   

// Number of channels (RP2040 supports max 4)
#define N_CH            2

// ADC GPIO pins (must be GPIO 26–29)
#define ADC_GPIOS       { 26, 27 }

// Linear conversion: value = m * volts + b
#define CONV_M          { 1.0f, 1.0f } // I do not know the actual conversions you'll need.
#define CONV_B          { 0.0f, 0.0f }

// ADC parameters 
#define ADC_VREF        3.3f
#define ADC_COUNTS_MAX  4095.0f

```
