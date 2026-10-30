"""Evidence-based, conservative LiDAR and tactile-strip analysis for survey annotations.

The results are *candidates*, never motor commands. A 90-bin scan cannot prove semantic
object identity. Unknown/no-return bins remain unknown rather than free space.
"""
from __future__ import annotations

import math
import time
from typing import Any

import numpy as np


def _pca_wall(ranges_cm: list[Any], indices: list[int], side: str) -> dict[str, Any] | None:
    points: list[tuple[float, float]] = []
    for idx in indices:
        if idx >= len(ranges_cm):
            continue
        value = ranges_cm[idx]
        if not isinstance(value, (int, float)) or value <= 0 or value > 300:
            continue
        r = float(value) / 100.0
        theta = math.radians(-4.0 * idx)  # bin 0 forward; increasing bins clockwise
        x, y = r * math.cos(theta), r * math.sin(theta)  # x forward, y left
        if side == "left" and y <= 0:
            continue
        if side == "right" and y >= 0:
            continue
        points.append((x, y))

    if len(points) < 6:
        return None
    arr = np.asarray(points, dtype=np.float64)
    centre = arr.mean(axis=0)
    centred = arr - centre
    cov = centred.T @ centred / max(1, len(points) - 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    tangent = eigvecs[:, int(np.argmax(eigvals))]
    if tangent[0] < 0:
        tangent = -tangent
    normal = np.asarray([-tangent[1], tangent[0]])
    projections = centred @ tangent
    residuals = centred @ normal
    span = float(projections.max() - projections.min())
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    distance = abs(float(np.dot(normal, centre)))
    angle = math.degrees(math.atan2(float(tangent[1]), float(tangent[0])))

    # Reject short line-like clusters, corner fragments, and poorly fitted geometry.
    if span < 0.45 or rms > 0.12 or distance < 0.18 or distance > 2.8:
        return None
    if abs(angle) > 40:
        return None
    count_score = min(1.0, len(points) / 12.0)
    span_score = min(1.0, span / 1.4)
    residual_score = max(0.0, 1.0 - rms / 0.12)
    confidence = 0.35 * count_score + 0.35 * span_score + 0.30 * residual_score
    return {
        "side": side,
        "distance_m": round(distance, 3),
        "line_angle_deg": round(angle, 2),
        "fit_rms_m": round(rms, 4),
        "span_m": round(span, 3),
        "valid_points": len(points),
        "confidence": round(confidence, 3),
        "classification": "possible_planar_wall",
    }


def analyse_lidar(sensor_snapshot: dict[str, Any] | None) -> dict[str, Any]:
    """Return wall/open-area/centreline evidence from the final 90-bin chassis-frame map."""
    if not sensor_snapshot:
        return {"valid": False, "reason": "no sensor snapshot"}
    lidar = sensor_snapshot.get("lidar") or {}
    ranges = lidar.get("d") or []
    if len(ranges) != 90:
        return {"valid": False, "reason": "expected 90 four-degree bins"}
    age = lidar.get("age_ms")
    if not isinstance(age, (int, float)) or age < 0 or age > 1000:
        return {"valid": False, "reason": "LiDAR map is stale or has no valid packet timestamp"}

    left = _pca_wall(ranges, list(range(54, 81)), "left")
    right = _pca_wall(ranges, list(range(9, 36)), "right")
    walls = [w for w in (left, right) if w is not None]
    result: dict[str, Any] = {
        "valid": True,
        "source": "LD20 rolling-filtered 90-bin chassis-frame map",
        "lidar_age_ms": int(age),
        "possible_walls": walls,
        "wall_prompt": None,
        "centreline": None,
        "open_area": False,
    }
    if walls:
        strongest = max(walls, key=lambda w: w["confidence"])
        result["wall_prompt"] = {
            "kind": "possible_wall",
            "side": "both" if left and right else strongest["side"],
            "confidence": strongest["confidence"],
            "evidence": walls,
            "message": "Possible planar wall geometry detected. This is not semantic wall recognition.",
        }
    if left and right:
        angle_delta = abs(left["line_angle_deg"] - right["line_angle_deg"])
        angle_delta = min(angle_delta, abs(180.0 - angle_delta))
        corridor_width = left["distance_m"] + right["distance_m"]
        if angle_delta <= 15 and 0.45 <= corridor_width <= 4.0:
            # Left wall is +y, right wall is -y. Positive means the path centre lies left of rover.
            offset = (left["distance_m"] - right["distance_m"]) / 2.0
            angle = (left["line_angle_deg"] + right["line_angle_deg"]) / 2.0
            confidence = min(left["confidence"], right["confidence"]) * max(0.0, 1.0 - angle_delta / 30.0)
            result["centreline"] = {
                "kind": "two_boundary_midline_candidate",
                "lateral_offset_m": round(offset, 3),
                "heading_error_deg": round(angle, 2),
                "estimated_width_m": round(corridor_width, 3),
                "confidence": round(confidence, 3),
                "support": ["left planar fit", "right planar fit"],
                "motion_authorized": False,
            }
            result["wall_prompt"]["message"] = "Possible corridor walls detected on both sides. Operator confirmation is required; no motion is authorized."
            return result

    # Only report a possible open area if a meaningful number of *valid measured*
    # forward bins have long ranges. Zero/no-return bins are never counted as free.
    front_indices = list(range(82, 90)) + list(range(0, 9))
    front_valid_clear = [
        float(ranges[i]) for i in front_indices
        if isinstance(ranges[i], (int, float)) and 200 <= float(ranges[i]) <= 350
    ]
    if len(front_valid_clear) >= 10 and not walls:
        result["open_area"] = True
        result["centreline"] = None
        result["open_area_prompt"] = {
            "kind": "possible_open_area_no_centreline",
            "valid_long_range_front_bins": len(front_valid_clear),
            "message": "Possible open paved area. Available LiDAR does not establish a reliable centreline; capture and inspect manually.",
            "suggested_action": "investigation_capture_or_manual_inspection",
            "motion_authorized": False,
        }
    return result


def detect_tactile_strip_jpeg(jpeg_bytes: bytes) -> dict[str, Any]:
    """Classical CV candidate detector for long, parallel rib-like strips.

    Requires optional opencv-python-headless. It deliberately reports candidates rather than
    asserting that a detected set of parallel edges is definitely a tactile guidance strip.
    """
    started = time.perf_counter()
    try:
        import cv2
    except Exception as exc:
        return {"available": False, "valid": False, "classification": "vision_unavailable", "detected": False, "confidence": 0.0, "reason": f"OpenCV unavailable: {exc}", "motion_authorized": False}

    data = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return {"available": True, "valid": False, "classification": "invalid_image", "detected": False, "confidence": 0.0, "reason": "JPEG decode failed", "motion_authorized": False}
    height, width = image.shape[:2]
    y0, y1 = int(height * 0.42), int(height * 0.96)
    roi = image[y0:y1, :]
    grey = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    grey = cv2.GaussianBlur(grey, (5, 5), 0)
    edges = cv2.Canny(grey, 45, 130)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180.0, threshold=max(18, width // 28),
        minLineLength=max(24, int((y1 - y0) * 0.12)), maxLineGap=12
    )

    candidates: list[tuple[float, float, float, float]] = []
    if lines is not None:
        for line in lines[:, 0, :]:
            x1, yy1, x2, yy2 = map(float, line)
            dx, dy = x2 - x1, yy2 - yy1
            length = math.hypot(dx, dy)
            if length < max(24, (y1 - y0) * 0.12) or abs(dy) < 1:
                continue
            angle_from_vertical = math.degrees(math.atan2(dx, abs(dy)))
            if abs(angle_from_vertical) > 22:
                continue
            candidates.append(((x1 + x2) / 2.0, angle_from_vertical, length, min(yy1, yy2)))

    # Merge Hough fragments that lie on the same image-space rib.
    candidates.sort(key=lambda q: q[0])
    groups: list[list[tuple[float, float, float, float]]] = []
    merge_px = max(5.0, width * 0.018)
    for item in candidates:
        if not groups or abs(item[0] - np.mean([x[0] for x in groups[-1]])) > merge_px:
            groups.append([item])
        else:
            groups[-1].append(item)

    ribs = []
    for group in groups:
        total = sum(x[2] for x in group)
        if total <= 0:
            continue
        ribs.append({
            "x": float(np.average([x[0] for x in group], weights=[x[2] for x in group])),
            "angle": float(np.average([x[1] for x in group], weights=[x[2] for x in group])),
            "length": total,
        })

    detected = False
    confidence = 0.0
    offset = None
    heading = None
    spacing_cv = None
    if len(ribs) >= 3:
        xs = np.asarray([r["x"] for r in ribs], dtype=np.float64)
        spacings = np.diff(xs)
        if len(spacings) >= 2 and float(np.mean(spacings)) > 2:
            spacing_cv = float(np.std(spacings) / np.mean(spacings))
            angle_std = float(np.std([r["angle"] for r in ribs]))
            coverage = min(1.0, sum(r["length"] for r in ribs) / max(1.0, 2.0 * (y1 - y0)))
            regularity = max(0.0, 1.0 - spacing_cv)
            parallelism = max(0.0, 1.0 - angle_std / 18.0)
            confidence = min(1.0, 0.40 * regularity + 0.35 * parallelism + 0.25 * coverage)
            if spacing_cv <= 0.75 and angle_std <= 12 and coverage >= 0.18 and confidence >= 0.48:
                detected = True
                centre_x = (float(xs.min()) + float(xs.max())) / 2.0
                offset = round((centre_x - width / 2.0) / (width / 2.0), 4)
                heading = round(float(np.mean([r["angle"] for r in ribs])), 2)

    latency_ms = (time.perf_counter() - started) * 1000.0
    return {
        "available": True,
        "valid": True,
        "classification": "possible_tactile_ribbed_strip" if detected else "not_detected_or_ambiguous",
        "detected": detected,
        "candidate_rib_count": len(ribs),
        "lateral_offset_normalized": offset,
        "heading_error_image_deg": heading,
        "spacing_coefficient_of_variation": None if spacing_cv is None else round(spacing_cv, 3),
        "confidence": round(confidence, 3),
        "frame_width": width,
        "frame_height": height,
        "roi": {"x": 0, "y": y0, "width": width, "height": y1 - y0},
        "processing_latency_ms": round(latency_ms, 2),
        "motion_authorized": False,
        "caveat": "Parallel rib-like edges may be tile joints or other textures; field validation and camera calibration are required.",
    }
