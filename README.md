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
        "ts": int,            # d[0]
        "current": float,     # d[1]
        "voltage": float,     # d[2]
    },
    "motor": {
        "ts": int,            # d[3]
        "throttle": float,    # d[4]
        "velocity": float,    # d[5]
    },
    "rpm_front": {
        "ts": int,            # d[6]
        "rpm_left": float,    # d[7]
        "rpm_right": float,   # d[8]
    },
    "rpm_back": {
        "ts": int,            # d[9]
        "rpm_left": float,    # d[10]
        "rpm_right": float,   # d[11]
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

| CS GPIO | Description          | Format                     |
|---------|----------------------|----------------------------|
| 22      | Power Monitor      | u32 ts + float current + float voltage    |
| 23      | Motor Controller      | u32 ts + float throttle + float velocity  |
| 24      | Front RPM       | u32 ts + float rpm_left + float rpm_right   |
| 25      | Back RPM        | u32 ts + float rpm_left + float rpm_right   |

Bus: `/dev/spidev0.0`
