import time
import struct
from multiprocessing import shared_memory, resource_tracker

"""
Import this file to read from shared memory
"""

SHM_NAME = "sensor_shm"

SENSOR_FMT = "<" + (
    "Q" +                              # global ts
    "I" + "f" + "f" +                  # power
    "I" + "f" + "f" +                  # steering
    "I" + "f" + "f" +                  # rpm_front
    "I" + "f" + "f" +                  # rpm_back
    "I" + "f" + "f" + "f" + "f" +      # gps
    "I" + "f" + "f"                    # motor
)
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)

SEQ_FMT = "<I"
SEQ_SIZE = struct.calcsize(SEQ_FMT)
BLOCK_SIZE = SEQ_SIZE + SENSOR_SIZE

def _read_seq(buf) -> int:
    return struct.unpack_from(SEQ_FMT, buf, 0)[0]

def format_snap(snap):
    lines = [f"seq: {snap['seq']}  ts: {snap['global_ts']}"]
    sensors = {
        "power":     ["current", "voltage"],
        "steering":  ["brake_pressure", "turn_angle"],
        "rpm_front": ["rpm_left", "rpm_right"],
        "rpm_back":  ["rpm_left", "rpm_right"],
        "gps":       ["gps_lat", "gps_long", "heading", "speed"],
        "motor":     ["rpm", "throttle"],
    }
    for sensor, fields in sensors.items():
        d = snap[sensor]
        vals = "  ".join(f"{f}: {d[f]:.3f}" if isinstance(d[f], float) else f"{f}: {d[f]}" for f in fields)
        lines.append(f"  {sensor:<12} ts: {d['ts']:<12} {vals}")
    return "\n".join(lines)

class SensorShmReader:
    """
    Attach to an existing POSIX shared memory block written by the SPI process.

    If the SHM block doesn't exist, the instance is created in an "unavailable"
    state and reads will return None.
    """
    def __init__(self, name: str = SHM_NAME):
        self.available = False
        self._shm = None
        self._buf = None

        try:
            shm = shared_memory.SharedMemory(name=name, create=False)
            resource_tracker.unregister(shm._name, "shared_memory")
        except FileNotFoundError:
            print("SHM not found. Please run C++ writer script.")
            return

        if shm.size < BLOCK_SIZE:
            shm.close()
            raise RuntimeError(f"SHM too small: {shm.size} < {BLOCK_SIZE}")

        self._shm = shm
        self._buf = shm.buf
        self.available = True

    def close(self):
        """Detach from the shared memory block."""
        if self._shm is not None:
            self._shm.close()
            self._shm = None
            self._buf = None
            self.available = False

    def read_snapshot(self):
        """
        Read a single consistent snapshot using a seq-lock protocol.
        Returns (seq:int, data:tuple) or None if unavailable.
        """
        if not self.available:
            return None

        buf = self._buf
        while True:
            seq1 = _read_seq(buf)
            if seq1 & 1:
                continue

            # unpack directly from the shared memory buffer (no slice/bytes needed)
            data = struct.unpack_from(SENSOR_FMT, buf, SEQ_SIZE)

            seq2 = _read_seq(buf)
            if seq1 == seq2 and not (seq2 & 1):
                return seq2, data

    def read_snapshot_dict(self):
        """
        Read a snapshot and return it as a structured dict, or None if unavailable.
        """
        snap = self.read_snapshot()
        if snap is None:
            return None

        seq, d = snap
        return {
            "seq": seq, "global_ts": d[0],
            "power": {"ts": d[1],  "current": d[2],  "voltage": d[3]},
            "steering": {"ts": d[4], "brake_pressure": d[5], "turn_angle": d[6]},
            "rpm_front": {"ts": d[7], "rpm_left": d[8], "rpm_right": d[9]},
            "rpm_back": {"ts": d[10], "rpm_left": d[11], "rpm_right": d[12]},
            "gps": {"ts": d[13], "gps_lat": d[14], "gps_long": d[15], "heading": d[16], "speed": d[17]},
            "motor": {"ts": d[18], "rpm": d[19], "throttle": d[20]},
        }

def main():
    import argparse

    parser = argparse.ArgumentParser(description="Read sensor shared memory")
    parser.add_argument(
        "mode",
        choices=["formatted", "unformatted"],
        nargs="?",
        default="formatted",
        help="Output mode"
    )
    args = parser.parse_args()

    RATE = 10
    PERIOD = 1 / RATE

    reader = SensorShmReader()
    if not reader.available:
        return 1

    try:
        while True:
            snap = reader.read_snapshot_dict()
            if snap is not None:
                if args.mode == "formatted":
                    print(format_snap(snap))
                elif args.mode == "formatted":
                    print(snap)
                else:
                    print("Usage: python reader.py [formatted | unformatted]")
            time.sleep(PERIOD)
    finally:
        reader.close()

if __name__ == "__main__":
    raise SystemExit(main())
