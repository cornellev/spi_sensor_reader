# uc26_sensor_reader

HDLC-framed SPI sensor reader → POSIX shared memory (seqlock) → Python reader.

This repository provides a C++ daemon that polls multiple SPI devices at ~200 Hz,
decodes HDLC-framed payloads with CRC checking, and publishes the latest sensor
snapshot into POSIX shared memory. Consumers (e.g. Python) can read the data
lock-free using a sequence lock.

The system is designed to be extended as additional sensors are brought online.

Also includes `spi_slave.c` to make embedded a bit easier for the team. Will update
this for easy ADC sensor integration soon.

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
    "gps": {                  # Currently Unimplemented
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
    sudo ./shm_writer
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

## Hardware

Current SPI setup (for Electrical reference):

| CS GPIO | Description          | Format                     |
|---------|----------------------|----------------------------|
| 22      | Power Monitor      | u32 ts + float current + float voltage    |
| 23      | Driver              | u32 ts + float throttle + float brake + float turn_angle  |
| 24      | Front RPM       | u32 ts + float rpm_left + float rpm_right   |
| 25      | Back RPM        | u32 ts + float rpm_left + float rpm_right   |

Bus: `/dev/spidev0.0`