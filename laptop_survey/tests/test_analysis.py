import math

from analysis import analyse_lidar, detect_tactile_strip_jpeg


def snapshot(ranges, age=20):
    return {"lidar": {"d": ranges, "age_ms": age, "ok": True}}


def test_stale_lidar_is_invalid():
    result = analyse_lidar(snapshot([250] * 90, age=1200))
    assert result["valid"] is False
    assert "stale" in result["reason"]


def test_no_returns_are_not_treated_as_open_space():
    result = analyse_lidar(snapshot([0] * 90))
    assert result["valid"] is True
    assert result["open_area"] is False
    assert result["centreline"] is None


def test_open_area_reports_no_centreline_without_walls():
    result = analyse_lidar(snapshot([250] * 90))
    assert result["valid"] is True
    assert result["open_area"] is True
    assert result["centreline"]["kind"] == "possible_open_area_no_centreline"
    assert result["centreline"]["motion_authorized"] is False


def test_parallel_lidar_walls_produce_midline_candidate():
    ranges = [0] * 90
    # Two ideal parallel walls at +/-1 m in the scanner's x-forward/y-left frame.
    for i in list(range(54, 81)) + list(range(9, 36)):
        theta = math.radians(-4.0 * i)
        if i >= 54:
            r = 1.0 / math.sin(theta)
        else:
            r = -1.0 / math.sin(theta)
        if 0.10 <= r <= 3.0:
            ranges[i] = int(round(r * 100))
    result = analyse_lidar(snapshot(ranges))
    assert result["valid"] is True
    assert len(result["possible_walls"]) >= 1
    assert result["centreline"] is not None
    assert result["centreline"]["kind"] == "two_boundary_midline_candidate"
    assert result["centreline"]["motion_authorized"] is False


def test_vision_detector_handles_invalid_jpeg_without_claiming_detection():
    result = detect_tactile_strip_jpeg(b"not a jpeg")
    assert result["valid"] is False
    assert result["classification"] in (None, "not_detected_or_ambiguous")
