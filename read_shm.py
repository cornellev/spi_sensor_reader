import time
import struct
from multiprocessing import shared_memory, resource_tracker

"""
Import this file to read from shared memory
"""

SHM_NAME = "sensor_shm"

SENSOR_FMT = "<" + (
    "I" + "f" + "f" +   # power
    "I" + "f" + "f" +   # motor
    "I" + "f" + "f" +   # rpm_front
    "I" + "f" + "f"     # rpm_back
)
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)
SEQ_SIZE = 4
BLOCK_SIZE = SEQ_SIZE + SENSOR_SIZE

def _read_seq(buf, offset=0) -> int:
    return int.from_bytes(buf[offset:offset + SEQ_SIZE], "little", signed=False)

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
            seq1 = _read_seq(buf, 0)
            if seq1 & 1:
                continue

            payload = bytes(buf[SEQ_SIZE:SEQ_SIZE + SENSOR_SIZE])

            seq2 = _read_seq(buf, 0)
            if seq1 == seq2 and not (seq2 & 1):
                return seq2, struct.unpack(SENSOR_FMT, payload)

    def read_snapshot_dict(self):
        """
        Read a snapshot and return it as a structured dict, or None if unavailable.
        """
        snap = self.read_snapshot()
        if snap is None:
            return None

        seq, d = snap

        return {
            "seq": seq,

            "power": {
                "ts": d[0],
                "current": d[1],
                "voltage": d[2],
            },

            "motor": {
                "ts": d[3],
                "throttle": d[4],
                "velocity": d[5],
            },

            "rpm_front": {
                "ts": d[6],
                "rpm_left": d[7],
                "rpm_right": d[8],
            },

            "rpm_back": {
                "ts": d[9],
                "rpm_left": d[10],
                "rpm_right": d[11],
            },
        }

def main():
    RATE = 10
    PERIOD = 1/RATE

    reader = SensorShmReader()
    if not reader.available:
        return 1

    try:
        while True:
            # You should usually use read_snapshot() instead for less overhead
            snap = reader.read_snapshot_dict()

            # snap should never be None here unless SHM disappeared mid-run
            if snap is not None:
                print(snap)

            time.sleep(PERIOD)

    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
