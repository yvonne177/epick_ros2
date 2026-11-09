# Robotiq ePick ROS 2 control and recording

A ROS 2 Jazzy backend and desktop GUI replacing the Robotiq Windows user interface
for direct Modbus RTU control. The GUI is a ROS client: another ROS 2 node can use
the same settings, services, and telemetry without the GUI.

## Launch the GUI

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths epick_ros2 --ignore-src -r -y
colcon build --packages-select epick_ros2
source install/setup.bash
ros2 launch epick_ros2 epick.launch.py
```

The launch starts the backend and GUI, **disconnected and not recording**. It does
not activate or grip automatically. A graphical Linux display (WSLg on Windows)
is required. For headless operation use `gui:=false`; to open only the GUI against
an existing backend, use `ros2 run epick_ros2 epick_gui`.

### COM3 and WSL2

Close the Windows Robotiq interface before connecting. Only the backend owns the
serial port; the GUI never accesses it directly.

**Windows COM3 is not automatically `/dev/ttyS3` in WSL2.** For a USB
RS-485 adapter, expose the adapter to WSL using
[USB/IP forwarding](https://learn.microsoft.com/windows/wsl/connect-usb), then
select its Linux device, commonly `/dev/ttyUSB0`. Check device permissions and
membership in the `dialout` group. An attached Windows application cannot keep
using that adapter at the same time.

The adapter verified in this workspace is an FTDI `0403:6015` USB RS-485
adapter. By default `device: auto` identifies a single matching adapter using
its USB VID/PID and prefers its stable by-id path. USB bus IDs can change after
replugging; confirm the adapter identity using `usbipd list` before attaching.

The backend discovers ports at startup, every two seconds, and on each Connect.
The GUI shows the detected adapter and offers **Refresh ports** and an editable
port dropdown. Leave the dropdown on **auto** to follow the adapter even when its
`ttyUSB` number changes. Discovery only enumerates USB metadata; Connect verifies
the Modbus response. It never automatically activates, grips, releases, or
reconnects after a connection loss.

If no matching adapter is visible, the GUI reports that it must be attached to
WSL. It does not silently select an unrelated serial device or request Windows
administrator access. If using a replacement adapter, change
`adapter_serial_number` or select its port manually. An empty serial-number filter
accepts a single FTDI `0403:6015` adapter; multiple matches require explicit selection.

Replace `<BUSID>` with the adapter's bus ID from `usbipd list`. The Windows
setup is:

```powershell
# Administrator PowerShell, one-time sharing:
usbipd bind --busid <BUSID>
# Normal PowerShell, attach for the current WSL session:
usbipd attach --wsl --busid <BUSID>
```

To compare against the Windows interface, first stop recording and call
`/epick/disconnect`, then run `usbipd detach --busid <BUSID>` in Windows and reconnect
the Windows interface to COM3. Reattach before using ROS again. This is a
**sequential comparison**, not simultaneous monitoring of the Windows GUI.
The backend's configured settings are not imported from the Windows application;
hardware echo/status fields report the actual device state.

```bash
ros2 launch epick_ros2 epick.launch.py device:=/dev/ttyUSB0
```

Default Robotiq serial settings are 115200 baud, 8N1, slave ID 9. Verify these
against the device's Windows connection settings. Set additional parameters using
[`epick.yaml`](./epick_ros2/config/epick.yaml) or a custom file with
`config:=/absolute/path/to/parameters.yaml`.

## Operator workflow

1. Leave the port on **auto** and click **Connect**. This finds the adapter and
   only reads status. Manual port selection remains available.
2. Click **Activate** if the status is not ready. Activation resets/reasserts the
   activation bit; do not do this while relying on the gripper to hold a load.
3. Choose automatic or advanced mode. In advanced mode set maximum/minimum vacuum,
   grip timeout, release delay, and optionally continuous grip.
4. Click **Apply settings** to save defaults, or **Grip** / **Release** to save and
   send them. Settings do not move the device until a command is issued.
5. Click **Start recording**, perform the experiment, then **Stop & save**.

The panel displays measured vacuum over the last 60 seconds, differential
pressure, activation, object detection, actuator state, faults, recording state,
and the output directory. Failed requests are shown explicitly. Telemetry older
than two seconds is marked stale.
The color-changing flag beside the activation status is red below **60% vacuum**:
**Suction below 60% - readjust connection**. From **60% to below 74%** it is yellow:
**Suction 60-<74% - potential suction failure; readjust connection**.
This warning indicates low suction, not a prediction of when failure will occur.
At **74% or above** it turns green
and reports that the suction threshold is met. If the device is not activated or
reports a fault, a threshold-met reading is yellow instead. Disconnected or stale
telemetry makes the flag gray/unknown, never green. This is a display-only threshold,
not independent proof of a secure hold; it does not change pump commands or settings.
Numbered sections separate connection/activation, command settings, gripper
actions, and recording. Grip and release settings are grouped separately beside
the white-background live graph; **Apply settings** saves defaults without motion.
**Restart graph** clears only the plotted history; live samples resume immediately
without changing the device or interrupting bag/CSV recording.

### Units and behavior

- Vacuum is displayed as Robotiq's `% vacuum`, numerically equal to kPa below
  ambient in the protocol. Differential pressure is `gPO - 100` kPa; vacuum is
  `100 - gPO`. No samples are synthesized when disconnected.
- Advanced maximum vacuum request: 10–100%; minimum vacuum: positive and below
  maximum. The device's achievable maximum is about 78%; larger targets can keep
  the pump running without reaching the target. Maximum 100 or continuous grip
  sets `rPR=0` (pump always on).
- Grip timeout and release delay: 0–25.5 seconds in 0.1-second steps.
  Grip timeout 0 means no timeout. Release delay 0 returns to hold when ambient
  pressure is detected.
- Release pressure uses the GUI/protocol scale **100–255**, not differential kPa:
  100 requests ambient and 255 requests +155 kPa. The ROS `release_pressure`
  parameter and recorded settings remain differential kPa (0–155), so GUI 100
  is recorded as 0. **ePick cannot generate positive blow-off pressure**; exposing
  the protocol request does not add that physical capability.
- Defaults: maximum vacuum 100%, minimum 10%, grip timeout 25 s, release time
  0.1 s, and release protocol pressure 100 (ambient). These are startup defaults;
  restart the backend after rebuilding to load them.
- Automatic mode delegates thresholds/timeouts to the device; advanced settings
  are only effective in advanced mode.
- The ROS `/epick/hold` service clears regulation, stops the pump, and leaves valves holding. It is
  **not** an emergency stop or guaranteed load retention.
- Disconnect and shutdown do not issue grip/release commands. Consider device
  communication-timeout behavior and mechanically support loads before testing.
- Command success means Modbus acknowledgement, not successful object acquisition.
  Monitor object status, fault codes, and measured pressure for completion.
- After a grip timeout, check the suction seal and timeout before retrying Grip.
  The backend confirms regulation has stopped before reasserting it, so a retry
  does not depend on issuing Release first. It never automatically vents a load.
  Activate only enables the device; it does not start the vacuum pump.

## Recording output

Every explicit session creates a unique UTC-named directory under
`~/.ros/epick_data/` (editable in the GUI):

```text
epick_20261109T190000_123456Z/
  session.json       initial configuration and units
  telemetry.csv      timestamps, pressure/vacuum, all decoded fields,
                     full raw input bytes, and configured command settings
  vacuum.csv         sample timestamps, elapsed seconds, vacuum (%), pressure (kPa)
  vacuum.png         vacuum level versus time for the full recording
  events.csv         command requests/results, settings, connection,
                     recording state, diagnostics and scalar topic events
  bag/               ROS 2 sqlite3 bag with all backend topics
```

The driver reads all eight input registers; every defined input field is decoded
and all reserved bytes are retained in `raw_hex`. The device does not report
every setting back: CSV `settings` records the backend's configured defaults,
while `request_echo` and the other decoded echoes come from the hardware.
All backend service commands and their results are published as events. Commands
made before recording starts are not included. Session configuration is captured
at start and parameter changes are recorded throughout the session.

Recording can start disconnected; CSV telemetry will remain empty until real
samples arrive. Vacuum samples are flushed to `vacuum.csv` as they arrive;
elapsed seconds are measured from the first recorded sample. **Stop & save**
also exports `vacuum.png` with every recorded sample, not just the live graph's
last 60 seconds. **Restart graph** does not clear recorded data or the saved graph.
The PNG is generated by the backend without a display, including during normal
shutdown. Sessions without telemetry save an explicitly labeled empty graph.
Graph export errors are reported as recording failures; the CSV and bag remain
available. Stopping closes the bag so it can be inspected/replayed:

```bash
ros2 bag info ~/.ros/epick_data/<session>/bag
ros2 bag play ~/.ros/epick_data/<session>/bag
```

Close the GUI with the window close button to be prompted to stop/save an active
recording. Stopping the launch also closes recording resources during normal
shutdown. Forced process termination or power loss can require `ros2 bag reindex`.
Closing only the GUI leaves a separately launched backend running; it does not
silently release a workpiece.

## ROS API for automation

The driver is a regular ROS node, **not a lifecycle node**. No configure/activate
lifecycle transitions are necessary. `/epick/activate` is a hardware command.

| Interface | Type | Meaning |
|---|---|---|
| `/epick/state` | `std_msgs/msg/String` | JSON: timestamps, all input fields, raw bytes and settings |
| `/epick/commands` | `std_msgs/msg/String` | JSON: action, timestamp, settings, success and result |
| `/epick/settings` | `std_msgs/msg/String` | Retained JSON parameter snapshot |
| `/epick/recording_state` | `std_msgs/msg/String` | Retained JSON active flag, path and error |
| `/epick/vacuum_pressure` | `std_msgs/msg/Float64` | Differential pressure in kPa, negative for vacuum |
| `/epick/status` | `std_msgs/msg/UInt16` | Raw first status byte |
| `/epick/connected` | `std_msgs/msg/Bool` | Retained verified connection state |
| `/epick/ports` | `std_msgs/msg/String` | Retained JSON port list, detected/connected device, and discovery error |
| `/epick/refresh_ports` | `std_srvs/srv/Trigger` | Refresh adapter discovery without opening serial ports |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Device, connection, and recording errors |
| `/epick/connect`, `/epick/reconnect`, `/epick/disconnect` | `std_srvs/srv/Trigger` | Explicit connection control |
| `/epick/activate`, `/epick/grip`, `/epick/release`, `/epick/hold` | `std_srvs/srv/Trigger` | Device commands |
| `/epick/start_recording`, `/epick/stop_recording` | `std_srvs/srv/Trigger` | Recording control |
| `/epick/set_vacuum` | `std_srvs/srv/SetBool` | Compatibility alias: true = grip, false = release |

Use the standard ROS parameter services to configure commands. For multiple
interdependent settings, use `set_parameters_atomically`; the GUI does this
before dispatching the command. Serial settings are editable only while
disconnected. Recording directory changes require recording to be stopped.

```bash
ros2 service call /epick/connect std_srvs/srv/Trigger '{}'
ros2 service call /epick/activate std_srvs/srv/Trigger '{}'
ros2 param set /epick_driver grip_timeout 5.0
ros2 service call /epick/start_recording std_srvs/srv/Trigger '{}'
ros2 service call /epick/grip std_srvs/srv/Trigger '{}'
ros2 service call /epick/release std_srvs/srv/Trigger '{}'
ros2 service call /epick/stop_recording std_srvs/srv/Trigger '{}'
```

## Protocol reference and validation

Implementation follows sections 4.2–4.7 of the
[Robotiq ePick instruction manual](https://s3.amazonaws.com/com-robotiq-website-prod-assets/website-assets/support_documents/document/EPick_Instruction_Manual_TM_20240322.pdf):
FC04 reads input registers 2000–2007, FC16 writes the three command registers
1000–1002, and the six payload bytes contain action, reserved bytes, `rPR`, `rSP`,
and `rFR`. The driver validates slave/function, byte counts, write acknowledgements,
timeouts and CRCs.

### Advanced controls and recording

The command mapping below was checked against the manual's sections 4.2–4.3
(pages 36–39). It verifies protocol compatibility, not the exact command sequence
of the proprietary Windows GUI; that requires a Windows serial capture or source.
Both the ROS GUI and service clients use the same backend command encoder.

| Control | ROS setting | Advanced-mode register encoding |
|---|---|---|
| Advanced mode | `advanced_mode` | `rMOD=1`; with activation and regulation, action byte `0x0B` |
| Grip maximum vacuum | `max_vacuum` | `rPR = 100 - max_vacuum`; continuous grip uses `rPR=0` |
| Grip minimum vacuum | `min_vacuum` | `rFR = 100 - min_vacuum` |
| Grip timeout | `grip_timeout` | `rSP = seconds × 10` |
| Release pressure | `release_pressure` | `rPR=100 + release_pressure`; GUI displays `rPR` (100–255), ROS stores differential kPa (0–155) |
| Release time | `release_delay` | `rSP = seconds × 10`; delay after ambient pressure is detected |

For example, maximum 75%, minimum 65%, and grip timeout 1.0 s produce
`0b 00 00 19 0a 23`. Release to ambient with a 1.7 s delay produces
`0b 00 00 64 11 23`. Grip/release first clear regulation and wait for confirmation
before reasserting it, including when retrying after a timeout.

The bag records all these settings at recording start and on changes in
`/epick/settings`, alongside each command/result in `/epick/commands` and each
sample in `/epick/state`. Measured vacuum is `vacuum_percent` in `/epick/state`;
measured differential pressure is `pressure_kpa` there and on
`/epick/vacuum_pressure`. The latter is negative during vacuum, not a positive
vacuum percentage. `release_pressure` and `release_delay` record the configured
target and delay, not measured release pressure or elapsed completion time.
The device does not echo the minimum-vacuum or timeout registers.

Tests use a pseudo-terminal Modbus device, not a real gripper:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest epick_ros2/test -q
```

Read-only hardware communication was verified on an FTDI `0403:6015` adapter at
115200 baud, 8N1, slave ID 9, including a short ROS/CSV/bag recording.
Grip/release command behavior and a Windows-GUI reading comparison have not yet
been hardware-validated. The presence of `/dev/ttyS3` alone does not verify COM3 access.
