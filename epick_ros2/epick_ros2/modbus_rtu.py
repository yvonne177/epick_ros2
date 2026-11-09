"""Validated Modbus RTU transport for Robotiq register blocks."""

from __future__ import annotations

import struct
import time
import math

import serial


class ModbusError(RuntimeError):
    """Raised when a Modbus request cannot be completed correctly."""


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class ModbusRtuClient:
    """Synchronous Modbus RTU client with strict response validation."""

    def __init__(
        self,
        device: str,
        baud_rate: int,
        parity: str,
        stop_bits: int,
        slave_id: int,
        timeout_seconds: float,
    ) -> None:
        if not 1 <= slave_id <= 247:
            raise ValueError("slave_id must be between 1 and 247")
        if parity not in {"N", "E", "O"}:
            raise ValueError("parity must be N, E, or O")
        if baud_rate <= 0 or stop_bits not in {1, 2}:
            raise ValueError("baud_rate must be positive and stop_bits must be 1 or 2")
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be finite and positive")

        self._device = device
        self._baud_rate = baud_rate
        self._parity = parity
        self._stop_bits = stop_bits
        self._slave_id = slave_id
        self._timeout_seconds = timeout_seconds
        self._serial: serial.Serial | None = None

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        if self.connected:
            return
        self._serial = serial.Serial(
            port=self._device,
            baudrate=self._baud_rate,
            parity=self._parity,
            stopbits=self._stop_bits,
            bytesize=serial.EIGHTBITS,
            timeout=self._timeout_seconds,
            write_timeout=self._timeout_seconds,
            exclusive=True,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def read_holding_register(self, address: int) -> int:
        response = self._request(0x03, struct.pack(">HH", address, 1), expected_data_length=3)
        if response[0] != 2:
            raise ModbusError(f"unexpected register byte count: {response[0]}")
        return struct.unpack(">H", response[1:])[0]

    def write_single_register(self, address: int, value: int) -> None:
        payload = struct.pack(">HH", address, value)
        response = self._request(0x06, payload, expected_data_length=4)
        if response != payload:
            raise ModbusError("write response does not echo the requested address and value")

    def read_input_bytes(self, address: int, count: int) -> bytes:
        if not 0 <= address <= 65535 or not 1 <= count <= 125 or address + count > 65536:
            raise ValueError("invalid input register range")
        response = self._request(
            0x04, struct.pack(">HH", address, count), expected_data_length=1 + count * 2
        )
        if response[0] != count * 2:
            raise ModbusError(f"unexpected register byte count: {response[0]}")
        return response[1:]

    def write_register_bytes(self, address: int, data: bytes) -> None:
        if not data or len(data) % 2 or len(data) > 246:
            raise ValueError("register data must contain 1 to 123 complete registers")
        count = len(data) // 2
        if not 0 <= address <= 65535 or address + count > 65536:
            raise ValueError("invalid output register range")
        expected = struct.pack(">HH", address, count)
        response = self._request(
            0x10, expected + bytes([len(data)]) + data, expected_data_length=4
        )
        if response != expected:
            raise ModbusError("write acknowledgement has incorrect address or count")

    def _request(self, function: int, payload: bytes, expected_data_length: int) -> bytes:
        if not self.connected or self._serial is None:
            raise ModbusError("serial connection is not open")

        request = bytes([self._slave_id, function]) + payload
        frame = request + struct.pack("<H", _crc16(request))
        try:
            # Above 19200 baud, the Modbus specification recommends a 1.75 ms gap.
            time.sleep(max(0.00175, 3.5 * 11 / self._baud_rate))
            self._serial.reset_input_buffer()
            if self._serial.write(frame) != len(frame):
                raise ModbusError("incomplete serial write")
            self._serial.flush()
            header = self._read_exactly(2)
            slave_id, response_function = header
            if slave_id != self._slave_id:
                raise ModbusError(f"response was for slave {slave_id}, expected {self._slave_id}")
            if response_function == function | 0x80:
                exception_code = self._read_exactly(1)[0]
                self._read_and_validate_crc(header + bytes([exception_code]))
                raise ModbusError(f"device returned Modbus exception {exception_code}")
            if response_function != function:
                raise ModbusError(
                    f"unexpected function {response_function:#x}, expected {function:#x}"
                )

            data = self._read_exactly(expected_data_length)
            self._read_and_validate_crc(header + data)
            return data
        except serial.SerialException as error:
            self.close()
            raise ModbusError(f"serial I/O failed: {error}") from error

    def _read_and_validate_crc(self, response_without_crc: bytes) -> None:
        crc = self._read_exactly(2)
        expected = struct.pack("<H", _crc16(response_without_crc))
        if crc != expected:
            raise ModbusError("response CRC does not match")

    def _read_exactly(self, count: int) -> bytes:
        if self._serial is None:
            raise ModbusError("serial connection is not open")
        data = bytearray()
        deadline = time.monotonic() + self._timeout_seconds
        while len(data) < count and time.monotonic() < deadline:
            chunk = self._serial.read(count - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != count:
            raise ModbusError(f"timed out waiting for {count} response bytes")
        return bytes(data)
