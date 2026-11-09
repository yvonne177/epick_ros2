import csv
import json

from matplotlib.figure import Figure
from matplotlib.image import imread
import pytest
from std_msgs.msg import String

from epick_ros2.protocol import Settings, decode_status
from epick_ros2.recording import Recorder, VACUUM_FIELDS


@pytest.mark.parametrize("samples", [
    [],
    [(0, 75)],
    [(0, 0), (1.5, 75), (65, 60), (120, -20)],
])
def test_vacuum_data_and_full_session_graph(tmp_path, monkeypatch, samples):
    recorder = Recorder()
    figures = []
    savefig = Figure.savefig

    def capture_figure(figure, *args, **kwargs):
        figures.append(figure)
        return savefig(figure, *args, **kwargs)

    monkeypatch.setattr(Figure, "savefig", capture_figure)
    path = recorder.start(str(tmp_path), {"/epick/state": "std_msgs/msg/String"}, {})
    try:
        for elapsed, vacuum in samples:
            stamp = 1_000_000_000 + int(elapsed * 1e9)
            state = decode_status(bytes([0, 0, 0, 100, 100 - vacuum] + [0] * 11))
            state.update(stamp_ns=stamp, timestamp_utc=f"sample-{elapsed}",
                         settings=Settings().to_dict())
            recorder.write("/epick/state", String(data=json.dumps(state)), stamp)
        with (path / "vacuum.csv").open() as file:
            reader = csv.DictReader(file)
            assert reader.fieldnames == VACUUM_FIELDS
            rows = list(reader)
        assert len(rows) == len(samples)  # available before Stop & save
        assert [float(row["elapsed_seconds"]) for row in rows] == [s[0] for s in samples]
        assert [int(row["vacuum_percent"]) for row in rows] == [s[1] for s in samples]
        assert [int(row["pressure_kpa"]) for row in rows] == [-s[1] for s in samples]
        assert [int(row["stamp_ns"]) for row in rows] == [
            1_000_000_000 + int(s[0] * 1e9) for s in samples]
        assert [row["timestamp_utc"] for row in rows] == [f"sample-{s[0]}" for s in samples]
    finally:
        recorder.stop()
    assert not recorder.active
    assert imread(path / "vacuum.png").shape == (750, 1500, 4)
    axes = figures[0].axes[0]
    assert axes.get_xlabel() == "Time since first sample (s)"
    assert axes.get_ylabel() == "Vacuum (%) / kPa below ambient"
    if samples:
        assert list(axes.lines[0].get_xdata()) == [s[0] for s in samples]
        assert list(axes.lines[0].get_ydata()) == [s[1] for s in samples]
    else:
        assert not axes.lines
        assert axes.texts[0].get_text() == "No vacuum samples recorded"
    recorder.stop()
    assert len(figures) == 1
    next_path = recorder.start(str(tmp_path), {}, {})
    recorder.stop()
    assert next_path != path
    with (next_path / "vacuum.csv").open() as file:
        assert list(csv.DictReader(file)) == []
    assert not figures[-1].axes[0].lines
