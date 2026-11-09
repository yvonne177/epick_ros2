"""Robotiq ePick command encoding and input register decoding."""

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Settings:
    advanced_mode: bool = True
    continuous_grip: bool = False
    max_vacuum: int = 100
    min_vacuum: int = 10
    grip_timeout: float = 25.0
    release_delay: float = 0.1
    release_pressure: int = 0

    def validate(self) -> None:
        if not 10 <= self.max_vacuum <= 100:
            raise ValueError("max_vacuum must be 10 to 100 (% vacuum); 100 requests continuous grip")
        if not 1 <= self.min_vacuum < self.max_vacuum:
            raise ValueError("min_vacuum must be positive and less than max_vacuum")
        for name in ("grip_timeout", "release_delay"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 <= value <= 25.5:
                raise ValueError(f"{name} must be 0 to 25.5 seconds")
            if not math.isclose(value * 10, round(value * 10), abs_tol=1e-8):
                raise ValueError(f"{name} must use 0.1 second increments")
        if not 0 <= self.release_pressure <= 155:
            raise ValueError("release_pressure must be 0 to 155 differential kPa (register 100 to 255)")

    def command(self, action: str) -> bytes:
        self.validate()
        mode = 0x02 if self.advanced_mode else 0
        action_byte = 0x01 | mode
        target = 100
        timeout = 0
        minimum = 100 - self.min_vacuum
        if action == "reset":
            action_byte = 0
        elif action == "activate":
            pass
        elif action == "hold":
            pass
        elif action in {"grip", "release"}:
            action_byte |= 0x08
            if action == "grip":
                target = 0 if self.continuous_grip else 100 - self.max_vacuum
                timeout = round(self.grip_timeout * 10)
            else:
                target = 100 + self.release_pressure
                timeout = round(self.release_delay * 10)
        else:
            raise ValueError(f"unknown action: {action}")
        return bytes([action_byte, 0, 0, target, timeout, minimum])

    def to_dict(self) -> dict:
        return asdict(self)


FAULTS = {
    0: "No fault",
    3: "Very porous material",
    5: "Action delayed",
    6: "Grip timeout",
    7: "Activation required",
    8: "Over temperature",
    9: "Communication timeout",
    10: "Under voltage",
    11: "Automatic release in progress",
    12: "Internal fault",
    15: "Automatic release completed",
}
OBJECTS = ("Unknown / regulating", "Object at minimum vacuum", "Object at maximum vacuum",
           "No object / lost / timeout")
ACTUATORS = ("Standby", "Gripping", "Passive release", "Active release")


def decode_status(data: bytes) -> dict:
    if len(data) != 16:
        raise ValueError("ePick status must contain all 8 input registers (16 bytes)")
    status, extension, faults, request_echo, pressure_raw = data[:5]
    fault = faults & 0x0F
    object_status = (status >> 6) & 0x03
    actuator_status = extension & 0x03
    return {
        "raw_hex": data.hex(),
        "status_raw": status,
        "status_extension_raw": extension,
        "fault_raw": faults,
        "activated": bool(status & 1),
        "mode": (status >> 1) & 3,
        "regulating": bool(status & 8),
        "activation_status": (status >> 4) & 3,
        "ready": bool(status & 1) and (status >> 4) & 3 == 3,
        "object_status": object_status,
        "object_description": OBJECTS[object_status],
        "actuator_status": actuator_status,
        "actuator_description": ACTUATORS[actuator_status],
        "fault_code": fault,
        "fault_description": FAULTS.get(fault, f"Unknown device fault {fault}"),
        "request_echo": request_echo,
        "pressure_raw": pressure_raw,
        "pressure_kpa": pressure_raw - 100,
        "vacuum_percent": 100 - pressure_raw,
    }
