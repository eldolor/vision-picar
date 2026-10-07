"""
body_state.py

The simulated body's state, shared between programs (PLAN-ros-alignment.md
3.36): `sim/body_server.py` (physics -- the one writer) publishes it, and
every `sim/sensor_server.py` worker (the readers) casts its sensors from it.
Shared memory rather than HTTP so a sensor read costs the physics program
nothing: it was the program near one core.

What is shared is exactly what the sensors depend on: the pose, the camera's
pan, the sim clock, which house, and the objects (furniture and movers are
solid to every sensor -- 3.9). The objects are re-encoded only when the dict
is REPLACED (GridWorld never mutates it in place -- 3.30), so a still house
costs one small struct per publish.

**A reader never sees a torn write.** The block carries a sequence number
(odd while a write is in progress) and a CRC32 of the payload; a reader
retries until it reads the same even sequence on both sides of its copy and
the CRC matches. The CRC is what makes this safe on a weakly ordered CPU
(the Jetson's Arm cores) without a cross-process lock: whatever order the
stores land in, a payload that does not match its own checksum is never
used.
"""

import json
import struct
import time
import zlib
from multiprocessing import shared_memory
from typing import Optional

SIZE = 1 << 20                     # 1 MiB: the furnished home's objects fit many times over
_HEAD = struct.Struct("<QI")       # sequence, payload length
_POSE = struct.Struct("<5dQ")      # x, y, theta, pan, sim_time (cells / rad / s), objects version
_CRC = struct.Struct("<I")
DEFAULT_NAME = "picar_sim_body"


def _attach(name: str) -> shared_memory.SharedMemory:
    """Attach WITHOUT adopting: before Python 3.13 the resource tracker
    registers every attach and unlinks the block when THAT process exits --
    a sensor worker restarting would delete the physics program's state out
    from under it. 3.13 has `track=False`; 3.10 (the Jetson) needs the
    unregister."""
    try:
        return shared_memory.SharedMemory(name=name, track=False)
    except TypeError:
        shm = shared_memory.SharedMemory(name=name)
        try:
            from multiprocessing import resource_tracker
            resource_tracker.unregister(shm._name, "shared_memory")
        except Exception:  # noqa: BLE001 -- best effort; the attach itself worked
            pass
        return shm


def encode_objects(objects: dict) -> bytes:
    return json.dumps([[x, y, name] for (x, y), name in sorted(objects.items())]).encode()


def decode_objects(raw: bytes) -> dict:
    return {(x, y): name for x, y, name in json.loads(raw)}


class StateWriter:
    """The physics program's side. One writer only."""

    def __init__(self, name: str = DEFAULT_NAME, house: str = ""):
        try:                                   # a stale block from a crashed run
            old = _attach(name)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass
        self.shm = shared_memory.SharedMemory(name=name, create=True, size=SIZE)
        self.name = name
        self._house = house.encode()
        self._seq = 0
        self._objects_ref = None
        self._objects_raw = b"[]"
        self._version = 0

    def publish(self, x, y, theta, pan, sim_time, objects: dict) -> None:
        if objects is not self._objects_ref:
            self._objects_ref = objects
            self._objects_raw = encode_objects(objects)
            self._version += 1
        payload = (_POSE.pack(x, y, theta, pan, sim_time, self._version)
                   + struct.pack("<H", len(self._house)) + self._house + self._objects_raw)
        body = payload + _CRC.pack(zlib.crc32(payload))
        buf = self.shm.buf
        self._seq += 1                                     # odd: writing
        _HEAD.pack_into(buf, 0, self._seq, len(payload))
        buf[_HEAD.size:_HEAD.size + len(body)] = body
        self._seq += 1                                     # even: done
        _HEAD.pack_into(buf, 0, self._seq, len(payload))

    def close(self) -> None:
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass


class StateReader:
    """A sensor worker's side. Cheap to call often; decodes the objects only
    when their version changes."""

    def __init__(self, name: str = DEFAULT_NAME, wait_s: float = 30.0):
        deadline = time.monotonic() + wait_s
        while True:
            try:
                self.shm = _attach(name)
                break
            except FileNotFoundError:
                if time.monotonic() > deadline:
                    raise RuntimeError(f"no simulated body state {name!r} -- start "
                                       "sim/body_server.py first (3.36)")
                time.sleep(0.1)
        self._version = None
        self._objects = {}

    def read(self, retries: int = 1000) -> Optional[dict]:
        buf = self.shm.buf
        for _ in range(retries):
            seq1, n = _HEAD.unpack_from(buf, 0)
            # n too short to hold the pose is a torn header: an empty payload
            # even "passes" the CRC (crc32(b"") is 0), and unpacking it raised
            # in sim/fake_lidar.py's loop on the Jetson (3.42).
            if seq1 == 0 or seq1 % 2 or n < _POSE.size or n + _HEAD.size + _CRC.size > SIZE:
                time.sleep(0.0002)
                continue
            raw = bytes(buf[_HEAD.size:_HEAD.size + n + _CRC.size])
            seq2, _ = _HEAD.unpack_from(buf, 0)
            payload, (crc,) = raw[:n], _CRC.unpack(raw[n:])
            if seq1 != seq2 or zlib.crc32(payload) != crc:
                continue
            x, y, theta, pan, sim_time, version = _POSE.unpack_from(payload, 0)
            off = _POSE.size
            (hl,) = struct.unpack_from("<H", payload, off)
            house = payload[off + 2:off + 2 + hl].decode()
            if version != self._version:
                self._objects = decode_objects(payload[off + 2 + hl:])
                self._version = version
            return {"x": x, "y": y, "theta": theta, "pan": pan, "sim_time": sim_time,
                    "house": house, "objects": self._objects, "seq": seq1}
        return None

    def close(self) -> None:
        self.shm.close()
