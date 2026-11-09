"""Single-owner Robotiq ePick backend shared by the GUI and other ROS nodes."""

from datetime import datetime, timezone
import json
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float64, String, UInt16
from std_srvs.srv import SetBool, Trigger

from epick_ros2.modbus_rtu import ModbusError, ModbusRtuClient
from epick_ros2.protocol import Settings, decode_status
from epick_ros2.recording import Recorder
from epick_ros2.discovery import serial_ports, select_port


class EpickDriver(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("epick_driver", **kwargs)
        defaults = {
            "device": "auto", "adapter_serial_number": "",
            "baud_rate": 115200, "parity": "N",
            "stop_bits": 1, "slave_id": 9, "poll_rate_hz": 10.0,
            "response_timeout_seconds": 0.25, "connect_on_start": False,
            "record_directory": "~/.ros/epick_data",
            **Settings().to_dict(),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._validate({name: self.get_parameter(name).value for name in defaults})
        self.add_on_set_parameters_callback(self._validate_changes)
        self.add_post_set_parameters_callback(self._parameters_changed)
        self._client = None
        self._last_state = None
        self._recorder = Recorder()
        self._last_error = ""
        self._resolved_device = ""
        self._topic_publishers = {}
        self._types = {}
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, cls, type_name, qos in [
            ("/epick/state", String, "std_msgs/msg/String", 100),
            ("/epick/commands", String, "std_msgs/msg/String", 100),
            ("/epick/settings", String, "std_msgs/msg/String", retained),
            ("/epick/recording_state", String, "std_msgs/msg/String", retained),
            ("/epick/connected", Bool, "std_msgs/msg/Bool", retained),
            ("/epick/ports", String, "std_msgs/msg/String", retained),
            ("/epick/vacuum_pressure", Float64, "std_msgs/msg/Float64", 10),
            ("/epick/status", UInt16, "std_msgs/msg/UInt16", 10),
            ("/diagnostics", DiagnosticArray, "diagnostic_msgs/msg/DiagnosticArray", 10),
        ]:
            self._topic_publishers[topic] = self.create_publisher(cls, topic, qos)
            self._types[topic] = type_name
        for action in ("refresh_ports", "connect", "reconnect", "disconnect", "activate", "grip", "release",
                       "hold", "start_recording", "stop_recording"):
            self.create_service(Trigger, f"/epick/{action}", self._service(action))
        self.create_service(SetBool, "/epick/set_vacuum", self._set_vacuum)
        self._timer = self.create_timer(1 / self.get_parameter("poll_rate_hz").value, self._poll)
        self._discovery_timer = self.create_timer(2.0, self._discover)
        self._publish_settings()
        self._publish_connected(False)
        self._publish_recording()
        self._discover()
        if self.get_parameter("connect_on_start").value:
            success, message = self._execute("connect")
            if not success:
                self.get_logger().error(message)

    def _settings(self) -> Settings:
        return Settings(**{name: self.get_parameter(name).value
                           for name in Settings.__dataclass_fields__})

    @staticmethod
    def _validate(values: dict) -> None:
        Settings(**{name: values[name] for name in Settings.__dataclass_fields__}).validate()
        if not values["device"]:
            raise ValueError("device must not be empty")
        if not 1 <= values["slave_id"] <= 247:
            raise ValueError("slave_id must be 1 to 247 (Robotiq default: 9)")
        if values["parity"] not in {"N", "E", "O"} or values["stop_bits"] not in {1, 2}:
            raise ValueError("parity must be N/E/O and stop_bits must be 1 or 2")
        if values["baud_rate"] <= 0:
            raise ValueError("baud_rate must be positive")
        for name in ("poll_rate_hz", "response_timeout_seconds"):
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if values["poll_rate_hz"] > 100:
            raise ValueError("poll_rate_hz must not exceed 100 Hz")
        if not values["record_directory"]:
            raise ValueError("record_directory must not be empty")

    def _validate_changes(self, parameters: list[Parameter]) -> SetParametersResult:
        values = {name: self.get_parameter(name).value for name in self.list_parameters([], 0).names}
        values.update({parameter.name: parameter.value for parameter in parameters})
        connection_fields = {
            "device", "baud_rate", "parity", "stop_bits", "slave_id",
            "response_timeout_seconds", "connect_on_start", "adapter_serial_number",
        }
        if self._client is not None and any(p.name in connection_fields for p in parameters):
            return SetParametersResult(successful=False, reason="disconnect before changing serial settings")
        if self._recorder.active and any(p.name == "record_directory" for p in parameters):
            return SetParametersResult(successful=False, reason="stop recording before changing its directory")
        try:
            self._validate(values)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _parameters_changed(self, parameters: list[Parameter]) -> None:
        if any(p.name == "poll_rate_hz" for p in parameters):
            self._timer.timer_period_ns = round(1e9 / self.get_parameter("poll_rate_hz").value)
        self._publish_settings()
        if any(p.name in {"device", "adapter_serial_number"} for p in parameters):
            self._discover()

    def _discover(self) -> dict:
        ports = []
        detected = ""
        error = ""
        try:
            ports = serial_ports()
            detected = select_port(ports, self.get_parameter("adapter_serial_number").value)
        except (OSError, ValueError) as problem:
            error = str(problem)
        result = {
            "ports": ports, "detected": detected, "error": error,
            "connected_device": self._resolved_device,
        }
        self._json("/epick/ports", result)
        return result

    def _publish(self, topic: str, message) -> None:
        self._topic_publishers[topic].publish(message)
        try:
            self._recorder.write(topic, message, self.get_clock().now().nanoseconds)
        except (OSError, RuntimeError) as error:
            try:
                self._recorder.stop()
            except (OSError, RuntimeError) as close_error:
                self.get_logger().error(f"Recording close failed: {close_error}")
            self._last_error = f"Recording failed: {error}"
            self.get_logger().error(self._last_error)
            self._publish_recording()
            self._report(self._last_error, DiagnosticStatus.ERROR)

    def _json(self, topic: str, data: dict) -> None:
        self._publish(topic, String(data=json.dumps(data, sort_keys=True)))

    def _publish_settings(self) -> None:
        self._json("/epick/settings", {
            "stamp_ns": self.get_clock().now().nanoseconds,
            "parameters": {name: self.get_parameter(name).value
                           for name in self.list_parameters([], 0).names},
        })

    def _publish_recording(self) -> None:
        self._json("/epick/recording_state", {
            "active": self._recorder.active,
            "path": str(self._recorder.path) if self._recorder.path else "",
            "error": self._last_error,
        })

    def _publish_connected(self, connected: bool) -> None:
        self._publish("/epick/connected", Bool(data=connected))

    def _report(self, message: str, level: bytes = DiagnosticStatus.OK) -> None:
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        diagnostic.status = [DiagnosticStatus(
            level=level, name="epick_driver", message=message,
            hardware_id=self.get_parameter("device").value,
            values=[KeyValue(key="recording", value=str(self._recorder.active))],
        )]
        self._publish("/diagnostics", diagnostic)

    def _service(self, action):
        def callback(request, response):
            del request
            response.success, response.message = self._execute(action)
            return response
        return callback

    def _set_vacuum(self, request, response):
        response.success, response.message = self._execute("grip" if request.data else "release")
        return response

    def _event(self, action: str, success: bool, message: str) -> None:
        self._json("/epick/commands", {
            "stamp_ns": self.get_clock().now().nanoseconds,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "action": action, "success": success, "message": message,
            "settings": self._settings().to_dict(),
        })

    def _execute(self, action: str) -> tuple[bool, str]:
        try:
            message = self._perform(action)
        except (ModbusError, OSError, ValueError, RuntimeError) as error:
            message = f"{action} failed: {error}"
            if action in {"start_recording", "stop_recording"}:
                self._last_error = message
                self._publish_recording()
            self._event(action, False, message)
            self._report(message, DiagnosticStatus.ERROR)
            return False, message
        self._event(action, True, message)
        return True, message

    def _perform(self, action: str) -> str:
        if action == "refresh_ports":
            result = self._discover()
            if result["error"]:
                raise ValueError(result["error"])
            return f"Detected ePick adapter: {result['detected']}"
        if action == "start_recording":
            self._last_error = ""
            path = self._recorder.start(
                self.get_parameter("record_directory").value, self._types,
                {"created_utc": datetime.now(timezone.utc).isoformat(),
                 "parameters": {name: self.get_parameter(name).value
                                for name in self.list_parameters([], 0).names},
                 "pressure_units": "kPa relative to ambient; vacuum_percent = -pressure_kpa",
                 "protocol": "Robotiq ePick Modbus RTU FC04/FC16"},
            )
            self._publish_settings()
            self._publish_connected(self._client is not None)
            self._discover()
            self._publish_recording()
            if not self._recorder.active:
                raise RuntimeError(self._last_error)
            return f"Recording to {path}"
        if action == "stop_recording":
            if not self._recorder.active:
                raise ValueError("no recording is running")
            self._event("stop_recording", True, "Finalizing recording")
            self._recorder.stop()
            self._publish_recording()
            return f"Recording saved to {self._recorder.path}"
        if action in {"connect", "reconnect"}:
            self._disconnect()
            device = self.get_parameter("device").value
            if device == "auto":
                result = self._discover()
                if result["error"]:
                    raise ValueError(result["error"])
                device = result["detected"]
            self._client = ModbusRtuClient(
                device=device,
                **{name: self.get_parameter(name).value
                   for name in ("baud_rate", "parity", "stop_bits", "slave_id")},
                timeout_seconds=self.get_parameter("response_timeout_seconds").value,
            )
            try:
                self._client.connect()
                state = self._read_state()
            except (ModbusError, OSError):
                self._disconnect()
                raise
            self._resolved_device = device
            self._publish_connected(True)
            self._discover()
            self._publish_state(state)
            self._report("Connected; no command sent")
            return f"Connected to {device}; verified ePick status; use Activate before grip/release"
        if action == "disconnect":
            self._disconnect()
            return "Disconnected; no release command sent"
        if self._client is None:
            raise ModbusError("not connected")
        state = self._read_state()
        settings = self._settings()
        if action == "activate":
            if state["ready"] and state["fault_code"] < 8:
                return "Gripper already activated"
            self._client.write_register_bytes(1000, settings.command("reset"))
            # Robotiq activation requires a rising edge of rACT.
            time.sleep(0.02)
            self._client.write_register_bytes(1000, settings.command("activate"))
        else:
            if not state["ready"]:
                raise ModbusError("gripper not activated; use Activate and wait for Ready")
            if state["fault_code"] >= 8:
                raise ModbusError(f"device fault: {state['fault_description']}")
            if action in {"grip", "release"}:
                # Reassert rGTO so retrying the same grip after a timeout works.
                self._client.write_register_bytes(1000, settings.command("hold"))
                deadline = time.monotonic() + 1.0
                while self._read_state()["regulating"]:
                    if time.monotonic() >= deadline:
                        raise ModbusError(
                            "device did not confirm regulation stopped; command not sent"
                        )
                    time.sleep(0.01)
            self._client.write_register_bytes(1000, settings.command(action))
        return f"{action.capitalize()} acknowledged; monitor telemetry for completion"

    def _read_state(self) -> dict:
        return decode_status(self._client.read_input_bytes(2000, 8))

    def _publish_state(self, state: dict) -> None:
        self._last_state = state
        self._json("/epick/state", {
            **state, "stamp_ns": self.get_clock().now().nanoseconds,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "settings": self._settings().to_dict(),
        })
        self._publish("/epick/vacuum_pressure", Float64(data=float(state["pressure_kpa"])))
        self._publish("/epick/status", UInt16(data=state["status_raw"]))
        self._report(state["fault_description"],
                     DiagnosticStatus.WARN if state["fault_code"] else DiagnosticStatus.OK)

    def _poll(self) -> None:
        if self._client is None:
            return
        try:
            self._publish_state(self._read_state())
        except (ModbusError, OSError) as error:
            self._disconnect()
            self._report(f"Telemetry failed; disconnected: {error}", DiagnosticStatus.ERROR)

    def _disconnect(self) -> None:
        client, self._client = self._client, None
        self._resolved_device = ""
        self._last_state = None
        if client is not None:
            client.close()
        self._publish_connected(False)

    def destroy_node(self):
        try:
            if self.context.ok():
                self._disconnect()
            elif self._client is not None:
                self._client.close()
                self._client = None
        finally:
            try:
                self._recorder.stop()
            finally:
                result = super().destroy_node()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EpickDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
