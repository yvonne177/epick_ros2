import os
import select
import struct
import threading
import time

import pytest

from epick_ros2.modbus_rtu import _crc16


class FakeEpick:
    """A wire-level test device; never uses a real serial port."""

    def __init__(self):
        self.master, self.slave = os.openpty()
        self.device = os.ttyname(self.slave)
        self.stop_event = threading.Event()
        self.writes = []
        self.requests = []
        self.bad_crc = False
        self.exception = False
        self.status = bytearray([0, 0, 0, 100, 100] + [0] * 11)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        buffer = bytearray()
        while not self.stop_event.is_set():
            readable, _, _ = select.select([self.master], [], [], 0.01)
            if not readable:
                continue
            buffer.extend(os.read(self.master, 4096))
            while len(buffer) >= 8:
                size = 9 + buffer[6] if buffer[1] == 0x10 else 8
                if len(buffer) < size:
                    break
                frame = bytes(buffer[:size])
                del buffer[:size]
                if _crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
                    continue
                self.requests.append(frame)
                slave, function = frame[:2]
                if self.exception:
                    response = bytes([slave, function | 0x80, 3])
                elif function == 0x04:
                    address, count = struct.unpack(">HH", frame[2:6])
                    assert address == 2000 and count == 8
                    response = bytes([slave, function, 16]) + self.status
                elif function == 0x10:
                    address, count = struct.unpack(">HH", frame[2:6])
                    assert address == 1000 and count == 3
                    data = frame[7:-2]
                    self.writes.append(data)
                    action = data[0]
                    self.status[0] = (0x30 if action & 1 else 0) | (action & 0x0F)
                    self.status[3] = data[3]
                    if action & 8:
                        self.status[4] = data[3]
                        self.status[0] |= 0x80 if data[3] < 100 else 0xC0
                        self.status[1] = 1 if data[3] < 100 else 2
                    else:
                        self.status[1] = 0
                    response = frame[:6]
                else:
                    response = bytes([slave, function | 0x80, 1])
                crc = _crc16(response)
                if self.bad_crc:
                    crc ^= 1
                os.write(self.master, response + struct.pack("<H", crc))

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def device():
    fake = FakeEpick()
    try:
        yield fake
    finally:
        fake.close()
