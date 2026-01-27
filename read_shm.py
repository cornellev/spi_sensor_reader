import time
import struct
from multiprocessing import shared_memory, resource_tracker

"""
Import this file to read from shared memory
"""

SHM_NAME = "sensor_shm"

SENSOR_FMT = "<" + (
    "I" + "3H" +        # sg1
    "I" + "3H" +        # sg2
    "I" + "3H" +        # sg3
    "I" + "f" + "f" +   # power
    "I" + "f" + "f"     # motor
)
SENSOR_SIZE = struct.calcsize(SENSOR_FMT)
SEQ_SIZE = 8
BLOCK_SIZE = SEQ_SIZE + SENSOR_SIZE

def _read_u64_le(buf, offset=0) -> int:
    return int.from_bytes(buf[offset:offset + 8], "little", signed=False)


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
            seq1 = _read_u64_le(buf, 0)
            if seq1 & 1:
                continue

            payload = bytes(buf[SEQ_SIZE:SEQ_SIZE + SENSOR_SIZE])

            seq2 = _read_u64_le(buf, 0)
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

            "sg1": {"ts": d[0],  "values": [d[1],  d[2],  d[3]]},
            "sg2": {"ts": d[4],  "values": [d[5],  d[6],  d[7]]},
            "sg3": {"ts": d[8],  "values": [d[9],  d[10], d[11]]},

            "power": {"ts": d[12], "current": d[13], "voltage": d[14]},
            "motor": {"ts": d[15], "throttle": d[16], "velocity": d[17]},
        }


def main():
    RATE = 200
    PERIOD = 1/RATE

    reader = SensorShmReader()
    if not reader.available:
        return 1

    try:
        while True:
            snap = reader.read_snapshot_dict()

            # snap should never be None here unless SHM disappeared mid-run
            if snap is not None:
                print(
                    f"seq={snap['seq']} | "
                    f"sg1(ts={snap['sg1']['ts']},[{snap['sg1']['values'][0]},{snap['sg1']['values'][1]},{snap['sg1']['values'][2]}]) "
                    f"sg2(ts={snap['sg2']['ts']},[{snap['sg2']['values'][0]},{snap['sg2']['values'][1]},{snap['sg2']['values'][2]}]) "
                    f"sg3(ts={snap['sg3']['ts']},[{snap['sg3']['values'][0]},{snap['sg3']['values'][1]},{snap['sg3']['values'][2]}]) | "
                    f"power(ts={snap['power']['ts']},I={snap['power']['current']:.3f},V={snap['power']['voltage']:.3f}) | "
                    f"motor(ts={snap['motor']['ts']},thr={snap['motor']['throttle']:.3f},vel={snap['motor']['velocity']:.3f})"
                )

            time.sleep(PERIOD)

    finally:
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
