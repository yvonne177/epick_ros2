"""Tk operator panel; all hardware access stays in the ROS backend."""

from collections import deque
import json
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from epick_ros2.protocol import Settings


class EpickGui:
    def __init__(self, root: tk.Tk, node: Node) -> None:
        self.root = root
        self.node = node
        self.connected = False
        self.recording = False
        self.busy = False
        self.closing = False
        self.closed = False
        self.pending = None
        self.request_deadline = 0.0
        self.initialized = False
        self.last_sample = None
        self.samples = deque(maxlen=12000)
        self.buttons = []
        self.variables = {}
        self.clients = {name: node.create_client(Trigger, f"/epick/{name}") for name in (
            "connect", "disconnect", "activate", "grip", "release", "hold",
            "start_recording", "stop_recording", "refresh_ports",
        )}
        self.parameters = AsyncParameterClient(node, "/epick_driver")
        self._build()
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, "/epick/connected", self._connection, retained)
        node.create_subscription(String, "/epick/ports", self._ports, retained)
        node.create_subscription(String, "/epick/state", self._state, 100)
        node.create_subscription(String, "/epick/settings", self._settings, retained)
        node.create_subscription(String, "/epick/recording_state", self._recording, retained)
        node.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostic, 10)
        root.protocol("WM_DELETE_WINDOW", self._close)
        self._tick()

    def _build(self) -> None:
        self.root.title("Robotiq ePick | ROS 2 control & recording")
        self.root.geometry("1160x880")
        self.root.minsize(1080, 840)
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", font=("TkDefaultFont", 10), background="#f3f6fa",
                        foreground="#203047")
        style.configure("TFrame", background="#f3f6fa")
        style.configure("TLabelframe", background="#f3f6fa", bordercolor="#d5dde7")
        style.configure("TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))
        style.configure("TButton", padding=(12, 7))
        style.configure("Primary.TButton", background="#1764b0", foreground="white")
        style.map("Primary.TButton", background=[("active", "#12518f"),
                                                 ("disabled", "#dce3ec")],
                  foreground=[("disabled", "#65758b")])
        style.configure("Muted.TLabel", foreground="#53657b")
        style.configure("Heading.TLabel", font=("TkDefaultFont", 22, "bold"))
        style.configure("Reading.TLabel", font=("TkDefaultFont", 12, "bold"))
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(3, weight=1)
        ttk.Label(outer, text="Robotiq ePick", style="Heading.TLabel").grid(
            row=0, column=0, sticky="w")
        ttk.Label(
            outer, text="Close the Windows app before connecting. Connect reads status only; "
            "Activate enables the device without starting the pump.",
            style="Muted.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(0, 8))
        connection = ttk.LabelFrame(outer, text="1  Connection & activation", padding=8)
        connection.grid(row=2, column=0, sticky="ew")
        connection.columnconfigure(0, weight=1)
        for column, (name, label, default, width) in enumerate((
            ("device", "Serial port (auto-detected)", "auto", 23),
            ("baud_rate", "Baud", "115200", 9),
            ("slave_id", "Slave ID", "9", 5),
        )):
            self.variables[name] = tk.StringVar(value=default)
            ttk.Label(connection, text=label).grid(row=0, column=column, sticky="w", padx=4)
            if name == "device":
                self.port_selector = ttk.Combobox(
                    connection, textvariable=self.variables[name], values=("auto",), width=width)
                self.port_selector.grid(row=1, column=column, padx=4, sticky="ew")
            else:
                ttk.Entry(connection, textvariable=self.variables[name], width=width).grid(
                    row=1, column=column, padx=4, sticky="w")
        self._button(connection, "Connect", lambda: self._apply("connect", connection=True)).grid(
            row=1, column=3, padx=4)
        self._button(connection, "Disconnect", lambda: self._call("disconnect")).grid(
            row=1, column=4, padx=4)
        self._button(connection, "Refresh ports", lambda: self._call("refresh_ports")).grid(
            row=1, column=5, padx=4)
        self._button(connection, "Activate", lambda: self._apply("activate")).grid(
            row=1, column=6, padx=4)
        self.connection_text = tk.StringVar(value="Waiting for ROS backend...")
        ttk.Label(connection, textvariable=self.connection_text, style="Reading.TLabel").grid(
            row=2, column=0, columnspan=7, sticky="w", pady=(8, 0))
        self.port_text = tk.StringVar(value="Detecting serial adapters...")
        ttk.Label(connection, textvariable=self.port_text, wraplength=1000,
                  style="Muted.TLabel").grid(row=3, column=0, columnspan=7, sticky="w")
        workspace = ttk.Frame(outer)
        workspace.grid(row=3, column=0, sticky="nsew", pady=8)
        workspace.columnconfigure(1, weight=1)
        workspace.rowconfigure(0, weight=1)
        controls = ttk.LabelFrame(workspace, text="2  Command settings", padding=8)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        controls.columnconfigure(0, weight=1)
        self.variables["advanced_mode"] = tk.BooleanVar(value=True)
        self.variables["continuous_grip"] = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text="Advanced mode",
                        variable=self.variables["advanced_mode"]).grid(row=0, column=0, sticky="w")
        ttk.Label(controls, text="Unchecked: device-managed automatic mode.",
                  style="Muted.TLabel").grid(row=1, column=0, sticky="w", pady=(0, 4))
        grip_settings = ttk.LabelFrame(controls, text="Grip", padding=6)
        grip_settings.grid(row=2, column=0, sticky="ew")
        release_settings = ttk.LabelFrame(controls, text="Release", padding=6)
        release_settings.grid(row=3, column=0, sticky="ew", pady=(4, 0))
        defaults = Settings().to_dict()
        for row, (name, label, lower, upper) in enumerate((
            ("max_vacuum", "Maximum vacuum (%)", 10, 100),
            ("min_vacuum", "Minimum vacuum (%)", 1, 99),
            ("grip_timeout", "Grip timeout (s)", 0, 25.5),
            ("release_delay", "Release time (s)", 0, 25.5),
            ("release_pressure_raw", "Pressure (protocol 100-255)", 100, 255),
        )):
            parent = release_settings if name in {"release_delay", "release_pressure_raw"} else grip_settings
            row = row - 3 if parent is release_settings else row
            default = 100 + defaults["release_pressure"] if name == "release_pressure_raw" else defaults[name]
            self.variables[name] = tk.StringVar(value=str(default))
            ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 12), pady=1)
            ttk.Spinbox(
                parent, from_=lower, to=upper,
                increment=0.1 if name in {"grip_timeout", "release_delay"} else 1,
                textvariable=self.variables[name], width=7,
            ).grid(row=row, column=1, sticky="e", pady=1)
        ttk.Checkbutton(grip_settings, text="Continuous grip (pump always ON)",
                        variable=self.variables["continuous_grip"]).grid(
                            row=3, column=0, columnspan=2, sticky="w", pady=(5, 0))
        ttk.Label(grip_settings, text="Max 100 = continuous pump; device max ~78%.\n"
                  "Timeout 0 = no timeout.", style="Muted.TLabel").grid(
            row=4, column=0, columnspan=2, sticky="w")
        ttk.Label(release_settings, text="100 = ambient; 255 = +155 kPa request.\n"
                  "ePick cannot generate positive blow-off.\n"
                  "Time is the delay after reaching ambient.",
                  style="Muted.TLabel").grid(row=2, column=0, columnspan=2, sticky="w", pady=(4, 0))
        self._button(controls, "Apply settings", lambda: self._apply()).grid(
            row=4, column=0, sticky="ew", pady=(6, 0))
        ttk.Label(controls, text="Saves defaults only; no movement.",
                  style="Muted.TLabel").grid(row=5, column=0, sticky="w", pady=(4, 0))

        monitor = ttk.LabelFrame(workspace, text="Live vacuum", padding=8)
        monitor.grid(row=0, column=1, sticky="nsew")
        self.telemetry_text = tk.StringVar(value="No live telemetry")
        ttk.Label(monitor, textvariable=self.telemetry_text, style="Reading.TLabel",
                  wraplength=600).pack(anchor="w")
        self.detail_text = tk.StringVar(value="Device state: unknown")
        ttk.Label(monitor, textvariable=self.detail_text, wraplength=600,
                  style="Muted.TLabel").pack(anchor="w", pady=(4, 12))
        self.canvas = tk.Canvas(monitor, height=180, background="white",
                                highlightthickness=1, highlightbackground="#d5dde7")
        self.canvas.pack(fill="both", expand=True)
        ttk.Label(monitor, text="Vacuum (%) / kPa below ambient  |  Last 60 seconds",
                  style="Muted.TLabel").pack(anchor="w", pady=(6, 10))
        actions = ttk.LabelFrame(monitor, text="3  Gripper & graph actions", padding=8)
        actions.pack(fill="x")
        for column, (text, action) in enumerate((
            ("Grip", "grip"), ("Release", "release"),
        )):
            actions.columnconfigure(column, weight=1)
            button = self._button(actions, text, lambda a=action: self._apply(a))
            if action == "grip":
                button.configure(style="Primary.TButton")
            button.grid(row=0, column=column, sticky="ew", padx=3)
        actions.columnconfigure(2, weight=1)
        self._button(actions, "Restart graph", self._restart_graph).grid(
            row=0, column=2, sticky="ew", padx=3)
        ttk.Label(actions, text="Grip / Release apply the settings at left.\n"
                  "Restart graph clears the display only; recording continues.",
                  style="Muted.TLabel").grid(
                      row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))

        recording = ttk.LabelFrame(outer, text="4  Recording  |  ROS bag + CSV", padding=8)
        recording.grid(row=4, column=0, sticky="ew")
        recording.columnconfigure(0, weight=1)
        self.variables["record_directory"] = tk.StringVar(value="~/.ros/epick_data")
        ttk.Entry(recording, textvariable=self.variables["record_directory"], width=45).grid(
            row=1, column=0, padx=3, sticky="ew")
        self._button(recording, "Browse", self._browse).grid(row=1, column=1, padx=3)
        self._button(recording, "Start recording",
                     lambda: self._apply("start_recording", recording=True)).grid(
                         row=1, column=2, padx=3)
        self._button(recording, "Stop & save",
                     lambda: self._call("stop_recording")).grid(row=1, column=3, padx=3)
        self.recording_text = tk.StringVar(value="Not recording")
        ttk.Label(recording, textvariable=self.recording_text, wraplength=950).grid(
            row=2, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self.message = tk.StringVar(value="Waiting for /epick_driver. No serial port is opened by this GUI.")
        ttk.Label(outer, textvariable=self.message, wraplength=1020).grid(
            row=5, column=0, sticky="w", pady=(6, 0))

    def _button(self, parent, text, command):
        button = ttk.Button(parent, text=text, command=command)
        self.buttons.append(button)
        return button

    def _browse(self) -> None:
        path = filedialog.askdirectory(parent=self.root)
        if path:
            self.variables["record_directory"].set(path)

    def _restart_graph(self) -> None:
        self.samples.clear()
        self._plot()
        self.message.set("Graph restarted. Device state and recording are unchanged.")

    def _connection(self, message: Bool) -> None:
        self.connected = message.data
        self.connection_text.set("Connected (status verified)" if self.connected else "Disconnected")
        if not self.connected:
            self.last_sample = None
            self.telemetry_text.set("Disconnected - no live telemetry")
            self.detail_text.set("Device state unknown; last plotted samples are historical")

    def _ports(self, message: String) -> None:
        data = json.loads(message.data)
        self.port_selector.configure(values=["auto"] + [port["path"] for port in data["ports"]])
        if data["connected_device"]:
            self.port_text.set(f"Connected port: {data['connected_device']}")
        elif data["detected"]:
            self.port_text.set(f"Detected: {data['detected']} (select auto, then Connect)")
        else:
            self.port_text.set(data["error"])

    def _state(self, message: String) -> None:
        data = json.loads(message.data)
        now = time.monotonic()
        self.last_sample = now
        self.samples.append((now, data["vacuum_percent"]))
        self.telemetry_text.set(
            f"Vacuum: {data['vacuum_percent']}%    Pressure: {data['pressure_kpa']} kPa    "
            f"{'READY' if data['ready'] else 'NOT ACTIVATED'}")
        self.detail_text.set(
            f"{data['object_description']} | {data['actuator_description']} | "
            f"Fault {data['fault_code']}: {data['fault_description']}")

    def _settings(self, message: String) -> None:
        parameters = json.loads(message.data)["parameters"]
        if not self.initialized:
            for name, variable in self.variables.items():
                if name == "release_pressure_raw":
                    variable.set(100 + parameters["release_pressure"])
                elif name in parameters:
                    variable.set(parameters[name])
            self.initialized = True
            self.message.set("Backend ready. Connect to read status, then Activate explicitly.")

    def _recording(self, message: String) -> None:
        data = json.loads(message.data)
        self.recording = data["active"]
        label = "RECORDING" if self.recording else "Stopped"
        self.recording_text.set(f"{label} | {data['path']}" if data["path"] else "Not recording")
        if data["error"]:
            self.message.set(data["error"])

    def _diagnostic(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "epick_driver" and status.level >= DiagnosticStatus.WARN:
                hint = (" - grip timed out before reaching vacuum. Check the suction seal "
                        "and timeout, then retry Grip; Activate does not start the pump."
                        if status.message == "Grip timeout" else "")
                self.message.set(status.message + hint)

    def _set_busy(self, busy: bool) -> None:
        self.busy = busy
        for button in self.buttons:
            button.configure(state="disabled" if busy else "normal")

    def _apply(self, action=None, connection=False, recording=False) -> None:
        if self.busy:
            return
        if not self.parameters.services_are_ready():
            self.message.set("ROS backend unavailable. Launch epick.launch.py first.")
            return
        try:
            if connection:
                values = {
                    "device": self.variables["device"].get().strip(),
                    "baud_rate": int(self.variables["baud_rate"].get()),
                    "slave_id": int(self.variables["slave_id"].get()),
                }
            elif recording:
                values = {"record_directory": self.variables["record_directory"].get().strip()}
            else:
                settings = Settings(
                    advanced_mode=self.variables["advanced_mode"].get(),
                    continuous_grip=self.variables["continuous_grip"].get(),
                    max_vacuum=int(self.variables["max_vacuum"].get()),
                    min_vacuum=int(self.variables["min_vacuum"].get()),
                    grip_timeout=float(self.variables["grip_timeout"].get()),
                    release_delay=float(self.variables["release_delay"].get()),
                    release_pressure=int(self.variables["release_pressure_raw"].get()) - 100,
                )
                settings.validate()
                values = settings.to_dict()
        except (ValueError, tk.TclError) as error:
            self.message.set(f"Invalid settings: {error}")
            return
        self._set_busy(True)
        future = self.parameters.set_parameters_atomically(
            [Parameter(name, value=value) for name, value in values.items()])
        self.pending = future
        self.request_deadline = time.monotonic() + 10

        def done(result):
            if result.cancelled():
                return
            self.pending = None
            self._set_busy(False)
            try:
                response = result.result()
            except Exception as error:
                # Exceptions arriving through ROS futures must be visible to the operator.
                self.node.get_logger().error(f"Parameter request failed: {error}")
                self.message.set(f"Parameter request failed: {error}")
                return
            if not response.result.successful:
                self.message.set(f"Settings rejected: {response.result.reason}")
            elif action:
                self._call(action)
            else:
                self.message.set("Settings saved. They will be sent with the next command.")
        future.add_done_callback(done)

    def _call(self, action: str) -> None:
        if self.busy:
            return
        client = self.clients[action]
        if not client.service_is_ready():
            self.message.set(f"Backend service /epick/{action} is unavailable")
            return
        self._set_busy(True)
        self.message.set(f"{action.replace('_', ' ').capitalize()} requested...")
        future = client.call_async(Trigger.Request())
        self.pending = future
        self.request_deadline = time.monotonic() + 10

        def done(result):
            if result.cancelled():
                return
            self.pending = None
            self._set_busy(False)
            try:
                response = result.result()
            except Exception as error:
                self.node.get_logger().error(f"{action} request failed: {error}")
                self.message.set(f"{action} request failed: {error}")
                self.closing = False
                return
            self.message.set(("OK: " if response.success else "FAILED: ") + response.message)
            if self.closing and action == "stop_recording":
                if response.success:
                    self._finish_close()
                else:
                    self.closing = False
        future.add_done_callback(done)

    def _plot(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 100:
            return
        left, top, bottom, right = 48, 20, height - 27, width - 18
        now = time.monotonic()
        visible = [(stamp, value) for stamp, value in self.samples if stamp >= now - 60]
        low = min([0] + [value for _, value in visible])
        low = (int(low) // 20) * 20
        high = 100

        def y(value):
            return bottom - (value - low) / (high - low) * (bottom - top)

        for value in range(low, high + 1, 20):
            canvas.create_line(left, y(value), right, y(value), fill="#e1e7ef")
            canvas.create_text(left - 8, y(value), text=str(value), fill="#53657b", anchor="e")
        canvas.create_text(left, height - 10, text="-60 s", fill="#53657b", anchor="w")
        canvas.create_text(right, height - 10, text="now", fill="#53657b", anchor="e")
        canvas.create_line(left, top, left, bottom, right, bottom, fill="#94a3b8")
        points = []
        for stamp, value in visible:
            points.extend([left + (stamp - (now - 60)) / 60 * (right - left), y(value)])
        if len(points) >= 4:
            canvas.create_line(*points, fill="#1764b0", width=2, tags="vacuum_trace")
        elif len(points) == 2:
            x, yy = points
            canvas.create_oval(x - 2, yy - 2, x + 2, yy + 2, fill="#1764b0",
                               outline="#1764b0", tags="vacuum_trace")
        else:
            canvas.create_text((left + right) / 2, (top + bottom) / 2,
                               text="Waiting for live vacuum samples", fill="#53657b")

    def _tick(self) -> None:
        if self.closed:
            return
        try:
            for _ in range(16):
                rclpy.spin_once(self.node, timeout_sec=0)
                if self.closed:
                    return
        except (ExternalShutdownException, KeyboardInterrupt):
            self._finish_close()
            return
        if self.closed:
            return
        if self.pending is not None and time.monotonic() > self.request_deadline:
            self.pending.cancel()
            self.pending = None
            self.closing = False
            self._set_busy(False)
            self.message.set("ROS request timed out; its outcome is unknown. "
                             "Check backend/device state before retrying.")
        if self.last_sample is not None and time.monotonic() - self.last_sample > 2:
            self.telemetry_text.set("STALE telemetry (>2 s) - do not rely on the last device state")
        self._plot()
        self.root.after(40, self._tick)

    def _close(self) -> None:
        if self.busy:
            self.message.set("Wait for the current request before closing.")
            return
        if self.recording:
            if messagebox.askyesno("Recording is active", "Stop and save recording before closing?",
                                   parent=self.root):
                self.closing = True
                self._call("stop_recording")
            return
        self._finish_close()

    def _finish_close(self) -> None:
        self.closed = True
        self.root.destroy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("epick_gui")
    root = None
    try:
        root = tk.Tk()
        EpickGui(root, node)
        root.mainloop()
    except KeyboardInterrupt:
        if root is not None:
            root.destroy()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
