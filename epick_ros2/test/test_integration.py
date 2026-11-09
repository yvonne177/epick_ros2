import csv
import json
import os
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64, String
import rosbag2_py

from epick_ros2.driver import EpickDriver


@pytest.fixture
def ros_context():
    # Keep simulated command services separate from any live driver on domain 0.
    rclpy.init(domain_id=173)
    try:
        yield
    finally:
        rclpy.shutdown()


@pytest.fixture
def backend(device, tmp_path, ros_context):
    driver = EpickDriver(parameter_overrides=[
        Parameter("device", value=device.device),
        Parameter("record_directory", value=str(tmp_path)),
    ])
    node = Node("epick_test_client")
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(node)
    try:
        yield driver, node, executor
    finally:
        executor.remove_node(node)
        executor.remove_node(driver)
        node.destroy_node()
        driver.destroy_node()
        executor.shutdown()


def wait_future(executor, future):
    executor.spin_until_future_complete(future, timeout_sec=5)
    assert future.done(), "ROS request did not complete"
    return future.result()


def call(node, executor, action):
    client = node.create_client(Trigger, f"/epick/{action}")
    try:
        assert client.wait_for_service(timeout_sec=3)
        return wait_future(executor, client.call_async(Trigger.Request()))
    finally:
        node.destroy_client(client)


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path / "bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    messages = []
    while reader.has_next():
        messages.append(reader.read_next())
    return messages


def test_ros_control_and_real_recording(backend, device):
    driver, node, executor = backend
    assert device.requests == []  # no port access on startup
    assert not call(node, executor, "grip").success
    assert call(node, executor, "start_recording").success
    assert not call(node, executor, "start_recording").success
    assert call(node, executor, "connect").success
    assert device.writes == []  # connect must never activate the gripper
    assert not call(node, executor, "grip").success
    assert call(node, executor, "activate").success
    assert device.writes[:2] == [driver._settings().command("reset"),
                                driver._settings().command("activate")]
    parameters = AsyncParameterClient(node, "/epick_driver")
    assert parameters.wait_for_services(timeout_sec=3)
    rejected = wait_future(executor, parameters.set_parameters_atomically([
        Parameter("max_vacuum", value=20), Parameter("min_vacuum", value=30)]))
    assert not rejected.result.successful
    assert driver._settings().max_vacuum == 100
    rejected = wait_future(executor, parameters.set_parameters_atomically([
        Parameter("slave_id", value=1)]))
    assert not rejected.result.successful
    result = wait_future(executor, parameters.set_parameters_atomically([
        Parameter("max_vacuum", value=75), Parameter("min_vacuum", value=65),
        Parameter("grip_timeout", value=1.0), Parameter("release_delay", value=1.7)]))
    assert result.result.successful
    expected_settings = {
        "advanced_mode": True, "continuous_grip": False,
        "max_vacuum": 75, "min_vacuum": 65, "grip_timeout": 1.0,
        "release_delay": 1.7, "release_pressure": 0,
    }
    assert call(node, executor, "grip").success
    driver._poll()
    assert driver._last_state["vacuum_percent"] == 75
    assert device.writes[-1] == bytes.fromhex("0b0000190a23")
    assert call(node, executor, "release").success
    assert device.writes[-1] == bytes.fromhex("0b0000641123")
    driver._poll()
    assert driver._last_state["pressure_kpa"] == 0
    assert call(node, executor, "hold").success
    assert call(node, executor, "stop_recording").success
    assert not driver._recorder.active
    path = driver._recorder.path
    rows = list(csv.DictReader((path / "telemetry.csv").open()))
    assert rows and any(row["vacuum_percent"] == "75" for row in rows)
    assert any(json.loads(row["settings"]) == expected_settings
               and row["vacuum_percent"] == "75" for row in rows)
    assert all(len(row["raw_hex"]) == 32 for row in rows)
    assert all(row["stamp_ns"] and row["timestamp_utc"] for row in rows)
    metadata = json.loads((path / "session.json").read_text())
    assert metadata["parameters"]["slave_id"] == 9
    for name, value in {
        "advanced_mode": True, "max_vacuum": 100, "min_vacuum": 10,
        "grip_timeout": 25.0, "release_delay": 0.1, "release_pressure": 0,
    }.items():
        assert metadata["parameters"][name] == value
    events = list(csv.DictReader((path / "events.csv").open()))
    commands = [json.loads(row["payload"]) for row in events if row["topic"] == "/epick/commands"]
    assert any(event["action"] == "grip" and event["success"] for event in commands)
    assert any(event["action"] == "grip" and not event["success"] for event in commands)
    assert any(event["action"] == "stop_recording" for event in commands)
    for action in ("grip", "release"):
        assert any(event["action"] == action and event["success"]
                   and event["settings"] == expected_settings for event in commands)
    messages = read_bag(path)
    assert {topic for topic, _, _ in messages} >= {
        "/epick/state", "/epick/settings", "/epick/commands", "/diagnostics",
        "/epick/recording_state", "/epick/connected", "/epick/vacuum_pressure", "/epick/status",
    }
    states = [json.loads(deserialize_message(data, String).data)
              for topic, data, _ in messages if topic == "/epick/state"]
    assert any(state["vacuum_percent"] == 75 for state in states)
    assert any(state["vacuum_percent"] == 75 and state["pressure_kpa"] == -75
               and state["settings"] == expected_settings for state in states)
    assert any(state["vacuum_percent"] == 0 and state["pressure_kpa"] == 0
               and state["settings"] == expected_settings for state in states)
    snapshots = [json.loads(deserialize_message(data, String).data)["parameters"]
                 for topic, data, _ in messages if topic == "/epick/settings"]
    assert snapshots[0] == metadata["parameters"]
    assert any(all(snapshot[name] == value for name, value in expected_settings.items())
               for snapshot in snapshots)
    bag_commands = [json.loads(deserialize_message(data, String).data)
                    for topic, data, _ in messages if topic == "/epick/commands"]
    for action in ("grip", "release"):
        assert any(event["action"] == action and event["success"]
                   and event["settings"] == expected_settings for event in bag_commands)
    pressures = [deserialize_message(data, Float64).data
                 for topic, data, _ in messages if topic == "/epick/vacuum_pressure"]
    assert -75.0 in pressures and 0.0 in pressures
    assert (path / "bag" / "metadata.yaml").exists()
    writes = len(device.writes)
    assert call(node, executor, "disconnect").success
    assert len(device.writes) == writes
    assert call(node, executor, "start_recording").success
    assert driver._recorder.path != path
    assert call(node, executor, "stop_recording").success


def test_recording_errors_are_reported(backend, tmp_path):
    driver, node, executor = backend
    target = tmp_path / "not_a_directory"
    target.write_text("test")
    result = driver.set_parameters_atomically([Parameter("record_directory", value=str(target))])
    assert result.successful
    response = call(node, executor, "start_recording")
    assert not response.success
    assert "failed" in response.message
    assert not driver._recorder.active


def test_recording_write_failure_stops_and_finalizes(backend, monkeypatch):
    driver, node, executor = backend
    assert call(node, executor, "start_recording").success
    path = driver._recorder.path

    def failed_write(topic, message, stamp_ns):
        if driver._recorder.active:
            raise OSError("test disk write failure")

    monkeypatch.setattr(driver._recorder, "write", failed_write)
    driver._publish_settings()
    assert not driver._recorder.active
    assert "test disk write failure" in driver._last_error
    assert (path / "bag" / "metadata.yaml").exists()


def test_auto_connection_rediscovers_adapter(backend, device, monkeypatch, tmp_path):
    from epick_ros2 import driver as driver_module
    driver, node, executor = backend
    ports = []
    monkeypatch.setattr(driver_module, "serial_ports", lambda: ports)
    assert driver.set_parameters_atomically([
        Parameter("device", value="auto"),
        Parameter("adapter_serial_number", value="test-adapter"),
    ]).successful
    assert not call(node, executor, "connect").success
    assert device.requests == []
    ports.append({"device": device.device, "path": device.device, "vid": 0x0403,
                  "pid": 0x6015, "serial_number": "test-adapter"})
    assert call(node, executor, "refresh_ports").success
    assert device.requests == []  # discovery must not touch the serial port
    assert call(node, executor, "connect").success
    assert driver._resolved_device == device.device
    assert device.writes == []
    assert call(node, executor, "disconnect").success
    renamed = tmp_path / "renumbered-port"
    renamed.symlink_to(device.device)
    ports[0]["path"] = str(renamed)
    assert call(node, executor, "connect").success
    assert driver._resolved_device == str(renamed)
    assert driver.get_parameter("device").value == "auto"
    assert device.writes == []


def test_grip_retry_waits_for_stop_confirmation(backend, device, monkeypatch):
    driver, node, executor = backend
    assert call(node, executor, "connect").success
    assert call(node, executor, "activate").success
    assert call(node, executor, "grip").success
    device.status[2] = 6
    original_read = driver._read_state
    observations = []

    def delayed_stop():
        state = original_read()
        if device.writes[-1] == driver._settings().command("hold"):
            observations.append(len(device.writes))
            if len(observations) < 3:
                state["regulating"] = True
        return state

    monkeypatch.setattr(driver, "_read_state", delayed_stop)
    assert call(node, executor, "grip").success
    assert len(observations) == 3
    assert len(set(observations)) == 1
    assert device.writes[-1] == driver._settings().command("grip")
    assert not any(command[3] >= 100 and command[0] & 8 for command in device.writes)


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="requires a graphical display")
def test_gui_controls_plot_and_recording(backend, device, monkeypatch):
    import tkinter as tk
    from epick_ros2 import driver as driver_module
    from epick_ros2.gui import EpickGui

    driver, _, executor = backend
    ports = [{"device": device.device, "path": device.device, "vid": 0x0403,
              "pid": 0x6015, "serial_number": "test-adapter"}]
    monkeypatch.setattr(driver_module, "serial_ports", lambda: ports)
    assert driver.set_parameters_atomically([Parameter("device", value="auto")]).successful
    gui_node = Node("epick_test_gui")
    root = tk.Tk()
    gui = EpickGui(root, gui_node)

    def pump_until(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0)
            root.update()
            if predicate():
                return
            time.sleep(0.005)
        pytest.fail(f"GUI did not reach expected state: {gui.message.get()}")

    def click(text):
        next(button for button in gui.buttons if button.cget("text") == text).invoke()
        pump_until(lambda: not gui.busy)
        assert not gui.message.get().startswith(("FAILED", "Settings rejected")), gui.message.get()

    try:
        pump_until(lambda: gui.initialized)
        root.update_idletasks()
        assert gui.variables["max_vacuum"].get() == "100"
        assert gui.variables["min_vacuum"].get() == "10"
        assert gui.variables["grip_timeout"].get() == "25.0"
        assert gui.variables["release_delay"].get() == "0.1"
        assert gui.variables["release_pressure_raw"].get() == "100"
        assert gui.canvas.cget("background") == "white"
        for geometry in ("1080x840", "1160x880"):
            root.geometry(geometry)
            root.update_idletasks()
            for button in gui.buttons:
                assert button.winfo_width() >= button.winfo_reqwidth()
                assert button.winfo_rootx() >= root.winfo_rootx()
                assert button.winfo_rooty() >= root.winfo_rooty()
                assert (button.winfo_rootx() + button.winfo_width()
                        <= root.winfo_rootx() + root.winfo_width())
                assert (button.winfo_rooty() + button.winfo_height()
                        <= root.winfo_rooty() + root.winfo_height())
            assert gui.canvas.winfo_height() >= 180
        assert next(button for button in gui.buttons
                    if button.cget("text") == "Grip").cget("style") == "Primary.TButton"
        assert gui.variables["device"].get() == "auto"
        pump_until(lambda: device.device in gui.port_text.get())
        assert device.device in gui.port_selector.cget("values")
        click("Refresh ports")
        assert device.requests == []
        click("Connect")
        pump_until(lambda: gui.connected)
        assert device.writes == []
        click("Activate")
        click("Start recording")
        pump_until(lambda: gui.recording)
        click("Grip")
        assert device.writes[-1] == bytes.fromhex("0b000000fa5a")
        click("Release")
        assert device.writes[-1] == bytes.fromhex("0b000064015a")
        gui.variables["release_pressure_raw"].set("255")
        click("Release")
        assert device.writes[-1] == bytes.fromhex("0b0000ff015a")
        gui.variables["release_pressure_raw"].set("100")
        gui.variables["advanced_mode"].set(True)
        gui.variables["max_vacuum"].set("70")
        gui.variables["min_vacuum"].set("45")
        gui.variables["grip_timeout"].set("2.3")
        gui.variables["release_delay"].set("1.7")
        click("Grip")
        assert device.writes[-1] == bytes.fromhex("0b00001e1737")
        pump_until(lambda: len(gui.samples) >= 2 and gui.samples[-1][1] == 70)
        gui._plot()
        assert any(gui.canvas.type(item) == "line"
                   for item in gui.canvas.find_withtag("vacuum_trace"))
        assert "70%" in gui.telemetry_text.get()
        writes_before_restart = list(device.writes)
        recording_path = driver._recorder.path
        last_sample = gui.last_sample
        next(button for button in gui.buttons
             if button.cget("text") == "Restart graph").invoke()
        assert not gui.samples
        assert not gui.canvas.find_withtag("vacuum_trace")
        assert device.writes == writes_before_restart
        assert gui.last_sample == last_sample
        assert gui.recording and driver._recorder.active
        assert driver._recorder.path == recording_path
        assert not any(button.cget("text") == "Hold (stop pump)" for button in gui.buttons)
        pump_until(lambda: len(gui.samples) >= 2)
        gui._plot()
        assert gui.canvas.find_withtag("vacuum_trace")
        click("Release")
        assert device.writes[-1] == bytes.fromhex("0b0000641137")
        click("Stop & save")
        pump_until(lambda: not gui.recording)
        assert (driver._recorder.path / "bag" / "metadata.yaml").exists()
        assert len(list(csv.DictReader((driver._recorder.path / "telemetry.csv").open()))) > 0
        commands = [json.loads(deserialize_message(data, String).data)
                    for topic, data, _ in read_bag(driver._recorder.path)
                    if topic == "/epick/commands"]
        for action in ("grip", "release"):
            assert any(event["action"] == action and event["success"]
                       and event["settings"] == {
                           "advanced_mode": True, "continuous_grip": False,
                           "max_vacuum": 70, "min_vacuum": 45, "grip_timeout": 2.3,
                           "release_delay": 1.7, "release_pressure": 0,
                       } for event in commands)
        assert any(event["action"] == "release" and event["success"]
                   and event["settings"]["release_pressure"] == 155 for event in commands)
        click("Disconnect")
        pump_until(lambda: not gui.connected)
        ports.clear()
        driver._discover()
        pump_until(lambda: "No ePick FTDI adapter" in gui.port_text.get())
        assert tuple(gui.port_selector.cget("values")) == ("auto",)
    finally:
        gui._finish_close()
        gui_node.destroy_node()
