from types import SimpleNamespace

import pytest

from epick_ros2 import discovery


def port(path="/dev/ttyUSB0", serial="epick-test", vid=0x0403, pid=0x6015):
    return {
        "device": path, "path": path, "serial_number": serial,
        "vid": vid, "pid": pid, "description": "FTDI RS-485",
    }


def test_selects_identity_after_port_number_changes():
    assert discovery.select_port([port("/dev/ttyUSB4")], "epick-test") == "/dev/ttyUSB4"
    assert discovery.select_port([port("/dev/ttyUSB7")], "epick-test") == "/dev/ttyUSB7"
    assert discovery.select_port([port(), port("/dev/ttyUSB1", "other")], "epick-test") == "/dev/ttyUSB0"


def test_missing_wrong_or_ambiguous_adapters_are_not_selected():
    for ports, serial in [
        ([], ""), ([port(vid=0x1234)], ""), ([port(serial="other")], "epick-test"),
    ]:
        with pytest.raises(ValueError, match="No ePick FTDI adapter"):
            discovery.select_port(ports, serial)
    with pytest.raises(ValueError, match="Multiple matching"):
        discovery.select_port([port(), port("/dev/ttyUSB1", "other")], "")


def test_enumeration_uses_stable_alias_without_opening_ports(monkeypatch, tmp_path):
    alias = tmp_path / "usb-ftdi-test"
    alias.symlink_to("/dev/ttyUSB6")
    monkeypatch.setattr(discovery.Path, "glob", lambda self, pattern: iter([alias]))
    monkeypatch.setattr(discovery.list_ports, "comports", lambda: [
        SimpleNamespace(device="/dev/ttyUSB6", vid=0x0403, pid=0x6015,
                        serial_number="epick-test", description="RS-485"),
    ])
    ports = discovery.serial_ports()
    assert ports[0]["path"] == str(alias)
    assert discovery.select_port(ports, "epick-test") == str(alias)
