"""Serial adapter discovery without opening ports or issuing device commands."""

from pathlib import Path

from serial.tools import list_ports


def serial_ports() -> list[dict]:
    aliases = {}
    for path in sorted(Path("/dev/serial/by-id").glob("*")):
        aliases.setdefault(str(path.resolve()), str(path))
    return [
        {
            "device": port.device,
            "path": aliases.get(port.device, port.device),
            "description": port.description,
            "vid": port.vid,
            "pid": port.pid,
            "serial_number": port.serial_number or "",
        }
        for port in sorted(list_ports.comports(), key=lambda port: port.device)
    ]


def select_port(ports: list[dict], serial_number: str) -> str:
    candidates = [port for port in ports if port["vid"] == 0x0403 and port["pid"] == 0x6015]
    if serial_number:
        candidates = [port for port in candidates if port["serial_number"] == serial_number]
    if not candidates:
        identity = f" with serial {serial_number}" if serial_number else ""
        raise ValueError(
            f"No ePick FTDI adapter{identity} detected. "
            "Attach the USB RS-485 adapter to WSL using usbipd, then refresh."
        )
    if len(candidates) != 1:
        raise ValueError(
            "Multiple matching FTDI adapters detected; select a port explicitly "
            "or configure adapter_serial_number."
        )
    return candidates[0]["path"]
