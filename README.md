# spi_sensor_reader

C++ SPI reader → POSIX shared memory (seqlock) → Python reader  
~200 Hz sampling of strain gauges, power, and motor data.

Use this library to continuously poll sensor data and read it easily in Python.

Script should be continuously updated hook up more sensors.

## Python Data Structure

```python
{
    "seq": int,
    "power": {
        "ts": int,
        "current": float,
        "voltage": float,
    },
    "motor": {
        "ts": int,
        "throttle": float,
        "velocity": float,
    },
    "rpm_front": {
        "ts": int,
        "rpm_left": float,
        "rpm_right": float,
    },
    "rpm_back": {
        "ts": int,
        "rpm_left": float,
        "rpm_right": float,
    },
}
```

## Quick Start

1. **Compile & run the C++ writerBash**
   ```bash
   g++ -O2 -std=c++17 spi_shm.cpp \
      -lpigpiod_if2 -lrt -pthread \
      -o spi_writer

   ./spi_writer
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

| Slave | CS GPIO | Description          | Format                     |
|-------|---------|----------------------|----------------------------|
| 1     | 22      | Power       | u32 ts + float current + float voltage    |
| 2     | 23      | Motor       | u32 ts + float throttle + float velocity  |
| 3     | 24      | Front RPM       | u32 ts + float rpm_left + float rpm_right   |
| 4     | 25      | Back RPM        | u32 ts + float rpm_left + float rpm_right   |

Bus: `/dev/spidev0.0`
