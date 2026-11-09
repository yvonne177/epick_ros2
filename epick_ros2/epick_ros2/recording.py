"""Explicit recording sessions containing ROS bags, telemetry, and event CSVs."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message


TELEMETRY_FIELDS = [
    "stamp_ns", "timestamp_utc", "vacuum_percent", "pressure_kpa", "pressure_raw",
    "activated", "ready", "mode", "regulating", "activation_status",
    "object_status", "object_description", "actuator_status", "actuator_description",
    "fault_code", "fault_description", "request_echo", "status_raw",
    "status_extension_raw", "fault_raw", "raw_hex", "settings",
]


class Recorder:
    def __init__(self) -> None:
        self.path: Path | None = None
        self._bag = None
        self._files = []
        self._telemetry = None
        self._events = None

    @property
    def active(self) -> bool:
        return self._bag is not None

    def start(self, directory: str, topics: dict[str, str], metadata: dict) -> Path:
        if self.active:
            raise RuntimeError("recording is already running")
        root = Path(directory).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        path = root / datetime.now(timezone.utc).strftime("epick_%Y%m%dT%H%M%S_%fZ")
        path.mkdir()
        self.path = path
        complete = False
        try:
            (path / "session.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            telemetry_file = (path / "telemetry.csv").open("x", newline="", encoding="utf-8")
            self._files.append(telemetry_file)
            events_file = (path / "events.csv").open("x", newline="", encoding="utf-8")
            self._files.append(events_file)
            self._telemetry = csv.DictWriter(telemetry_file, fieldnames=TELEMETRY_FIELDS)
            self._telemetry.writeheader()
            self._events = csv.writer(events_file)
            self._events.writerow(["stamp_ns", "topic", "payload"])
            self._bag = rosbag2_py.SequentialWriter()
            self._bag.open(
                rosbag2_py.StorageOptions(uri=str(path / "bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""),
            )
            for index, (name, message_type) in enumerate(topics.items(), start=1):
                self._bag.create_topic(rosbag2_py.TopicMetadata(
                    id=index, name=name, type=message_type, serialization_format="cdr"
                ))
            for file in self._files:
                file.flush()
            complete = True
        finally:
            if not complete:
                self.stop()
        return path

    def write(self, topic: str, message, stamp_ns: int) -> None:
        if not self.active:
            return
        self._bag.write(topic, serialize_message(message), stamp_ns)
        if topic == "/epick/state":
            data = json.loads(message.data)
            data["settings"] = json.dumps(data["settings"], sort_keys=True)
            self._telemetry.writerow({name: data[name] for name in TELEMETRY_FIELDS})
        else:
            payload = getattr(message, "data", str(message))
            self._events.writerow([stamp_ns, topic, payload])
        for file in self._files:
            file.flush()

    def stop(self) -> None:
        bag, self._bag = self._bag, None
        try:
            if bag is not None:
                bag.close()
        finally:
            try:
                for file in self._files:
                    file.close()
            finally:
                self._files = []
                self._telemetry = None
                self._events = None
