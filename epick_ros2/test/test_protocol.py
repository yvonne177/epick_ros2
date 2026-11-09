import pytest

from epick_ros2.modbus_rtu import ModbusError, ModbusRtuClient, _crc16
from epick_ros2.protocol import Settings, decode_status


def test_documented_crc_and_command_bytes():
    assert _crc16(bytes.fromhex("090407d00002")) == 0x0E70
    settings = Settings(max_vacuum=75, min_vacuum=65, grip_timeout=1.0)
    assert settings.command("grip") == bytes.fromhex("0b0000190a23")
    assert settings.command("release") == bytes.fromhex("0b0000640123")
    assert settings.command("hold") == bytes.fromhex("030000640023")
    assert Settings(advanced_mode=False).command("grip")[0] == 9
    assert Settings(continuous_grip=True).command("grip")[3] == 0
    assert Settings(grip_timeout=25.5).command("grip")[4] == 255


@pytest.mark.parametrize("settings, grip, release", [
    (Settings(max_vacuum=78, min_vacuum=20, grip_timeout=1.0, release_delay=1.7),
     "0b0000160a50", "0b0000641150"),
    (Settings(max_vacuum=10, min_vacuum=1, grip_timeout=0.0, release_delay=25.5),
     "0b00005a0063", "0b000064ff63"),
    (Settings(continuous_grip=True, min_vacuum=20, grip_timeout=25.5),
     "0b000000ff50", "0b0000640150"),
    (Settings(), "0b000000fa5a", "0b000064015a"),
    (Settings(release_pressure=155), "0b000000fa5a", "0b0000ff015a"),
    (Settings(advanced_mode=False, max_vacuum=78, min_vacuum=20,
              grip_timeout=1.0, release_delay=1.7),
     "090000160a50", "090000641150"),
])
def test_manual_grip_release_register_mapping(settings, grip, release):
    assert settings.command("grip") == bytes.fromhex(grip)
    assert settings.command("release") == bytes.fromhex(release)


@pytest.mark.parametrize("changes", [
    {"max_vacuum": 101}, {"max_vacuum": 9}, {"min_vacuum": 100},
    {"min_vacuum": 0}, {"grip_timeout": 25.6}, {"grip_timeout": -0.1},
    {"grip_timeout": 0.15}, {"release_delay": float("nan")},
    {"release_pressure": -1}, {"release_pressure": 156},
])
def test_invalid_settings(changes):
    with pytest.raises(ValueError):
        Settings(**changes).validate()


def test_status_bits_and_units():
    state = decode_status(bytes.fromhex("bb01061e280000000000000000000000"))
    assert state["ready"]
    assert state["activated"] and state["regulating"]
    assert state["mode"] == 1
    assert state["object_status"] == 2
    assert state["actuator_status"] == 1
    assert state["fault_code"] == 6
    assert state["pressure_kpa"] == -60
    assert state["vacuum_percent"] == 60
    assert state["request_echo"] == 30
    with pytest.raises(ValueError):
        decode_status(bytes(6))


def make_client(device):
    return ModbusRtuClient(device.device, 115200, "N", 1, 9, 0.1)


def test_wire_protocol_and_bad_responses(device):
    client = make_client(device)
    client.connect()
    try:
        assert client.read_input_bytes(2000, 8) == bytes(device.status)
        command = Settings().command("grip")
        client.write_register_bytes(1000, command)
        assert device.writes == [command]
        assert client.read_input_bytes(2000, 8)[4] == 0
        device.bad_crc = True
        with pytest.raises(ModbusError, match="CRC"):
            client.read_input_bytes(2000, 8)
        device.bad_crc = False
        device.exception = True
        with pytest.raises(ModbusError, match="exception 3"):
            client.read_input_bytes(2000, 8)
    finally:
        client.close()


def test_read_timeout(device):
    client = make_client(device)
    client.connect()
    device.stop_event.set()
    device.thread.join()
    try:
        with pytest.raises(ModbusError, match="timed out"):
            client.read_input_bytes(2000, 8)
    finally:
        client.close()
