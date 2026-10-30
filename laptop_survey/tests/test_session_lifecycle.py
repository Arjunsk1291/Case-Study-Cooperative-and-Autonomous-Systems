import json
import time
import zipfile
from pathlib import Path

from manager import SurveyManager


class FakeResponse:
    def __init__(self, status_code=200, body=None):
        self.status_code = status_code
        self._body = body or {"ok": True}

    def json(self):
        return self._body

    def raise_for_status(self):
        if self.status_code >= 400:
            raise RuntimeError("HTTP error " + str(self.status_code))

    @property
    def text(self):
        return json.dumps(self._body)


def fake_payload(seq=10):
    sensors = {
        "snapshot_seq": seq,
        "snapshot_device_ms": seq * 250,
        "lidar": {"ok": True, "pps": 334, "crc": 0, "age_ms": 10, "d": [180] * 90},
        "imu": {
            "ok": True, "age_ms": 15, "yaw": 12.5, "pitch": 0.2, "roll": -0.1,
            "cal": [0, 3, 0, 0], "raw_valid": True,
            "accel_m_s2": [0.1, 0.2, 9.7], "gyro_rad_s": [0.0, 0.01, 0.0],
            "mag_uT": [20.0, 1.2, 41.0], "temperature_c": 28,
        },
        "gps": {"fix": False, "sats": 0, "hdop": 99.9, "age_ms": -1,
                "altitude_valid": False, "speed_valid": False, "course_valid": False,
                "utc_valid": False, "chars": 50},
        "us": [80, 65, 70, 90, 75],
        "usRaw": [81, 64, 70, 91, 75],
        "usFiltered": [80, 65, 70, 90, 75],
        "usValid": [True, True, True, True, True],
        "usAgeMs": [20, 20, 20, 20, 20],
        "usTimeouts": [0, 0, 0, 0, 0],
    }
    return {
        "state": "MANUAL",
        "cycle": {"min": 2, "avg": 3, "max": 5},
        "sensors": sensors,
        "values": [
            {"name": "sys.firmware", "value": "survey-test"},
            {"name": "sys.uptime_s", "value": "120"},
            {"name": "sys.heap_kB", "value": "180"},
            {"name": "sys.min_heap_kB", "value": "160"},
            {"name": "wifi.ap_clients", "value": "2"},
            {"name": "rover.ip", "value": "192.168.14.2"},
            {"name": "rover.connected", "value": "true"},
            {"name": "rover.voltage_V", "value": "11.5"},
            {"name": "rover.rtt_ms", "value": "90"},
            {"name": "drive.command_l", "value": "0"},
            {"name": "drive.command_r", "value": "0"},
            {"name": "drive.command_age_ms", "value": "20"},
        ],
    }


def test_survey_stop_keeps_logging_and_finalize_builds_verified_zip(tmp_path: Path):
    manager = SurveyManager("http://127.0.0.1:1", tmp_path, "test-token")
    manager.read_values = lambda: fake_payload(10)
    manager.camera_heartbeat({"board_id": "AI_THINKER_ESP32CAM_R04", "firmware": "R04-CAM-1.0.0", "camera_ready": True, "width": 320, "height": 240, "jpeg_quality": 12, "xclk_hz": 20000000, "psram_detected": False, "free_heap": 249544, "min_free_heap": 246372, "largest_free_block": 110580, "wifi_rssi": -45, "wifi_ip": "192.168.14.3", "frame_sequence": 0})
    assert manager.camera_online() is True
    manager.camera_preview_saved({"frame_id": "preview_1", "laptop_received_at": "2026-10-30T12:00:00Z"}, tmp_path / "preview.jpg")
    manager.box_request = lambda method, path, params=None, timeout=0.8: FakeResponse(
        200, {"ok": True, "drive_inhibited": True}
    )

    started = manager.start_session("tester", "campus survey")
    session_id = started["session"]["session_id"]
    assert started["session"]["state"] == "active"
    assert manager.estop_latched is True
    assert manager.drive_enabled is False

    manager._record_sample(fake_payload(10), "2026-10-30T12:00:00Z", time.monotonic_ns())
    assert manager.session["quality"]["sensor_snapshots"] == 1

    enabled = manager.enable_manual_drive()
    assert enabled["drive_enabled"] is True
    cmd = manager.drive(0.12, 0.12)
    assert cmd["ok"] is True
    assert cmd["left"] == 0.12

    capture = manager.capture("pillar and paving transition")
    assert capture["type"] == "INVESTIGATION_CAPTURE"
    annotation = manager.annotate({"tags": ["pillar_or_column", "cobblestone_rough_paving"],
                                   "notes": "Observed during manual survey"})
    assert annotation["session_id"] == session_id

    stop = manager.stop_rover("test_stop")
    assert stop["ok"] is True
    assert manager.session["state"] == "active"
    assert manager.session["quality"]["sensor_snapshots"] == 1

    # A new sensor sequence is accepted after ordinary Stop Rover; logging was not ended.
    manager._record_sample(fake_payload(11), "2026-10-30T12:00:01Z", time.monotonic_ns())
    assert manager.session["quality"]["sensor_snapshots"] == 2

    final = manager.finalize()
    assert final["session"]["state"] == "ready"
    assert final["session"]["images"] == 0  # Camera offline is allowed.
    archive = manager.archive(session_id)
    assert archive.exists() and archive.stat().st_size > 0

    with zipfile.ZipFile(archive) as zf:
        assert zf.testzip() is None
        root = session_id + "/"
        manifest = json.loads(zf.read(root + "manifest.json"))
        quality = json.loads(zf.read(root + "integrity/data_quality_report.json"))
        assert manifest["session_id"] == session_id
        assert quality["sensor_snapshots"] == 2
        assert root + "telemetry/sensor_snapshots.jsonl" in zf.namelist()
        assert root + "annotations/annotations.jsonl" in zf.namelist()
        assert root + "camera/frame_index.csv" in zf.namelist()
