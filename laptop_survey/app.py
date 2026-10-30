from __future__ import annotations
import asyncio, hashlib, hmac, json, os, re, uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request, Header
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image, UnidentifiedImageError

from manager import SurveyManager
from analysis import detect_tactile_strip_jpeg

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
DATA_ROOT = Path(os.environ.get("CAS_SURVEY_DATA", str(ROOT / "survey_data")))
BOX_URL = os.environ.get("CAS_BOX_URL", "http://192.168.17.1")
CAMERA_TOKEN = os.environ.get("CAS_CAMERA_TOKEN", "CHANGE_ME_CAMERA_TOKEN")
MAX_IMAGE_BYTES = int(os.environ.get("CAS_MAX_JPEG_BYTES", str(3 * 1024 * 1024)))
MAX_DRIVE = 0.25

manager = SurveyManager(BOX_URL, DATA_ROOT, CAMERA_TOKEN)
app = FastAPI(title="CAS B3/R07 Survey Collector", version="0.1.0")
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.on_event("startup")
def startup():
    manager.start_worker()


@app.on_event("shutdown")
def shutdown():
    manager.shutdown()


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/health")
def health():
    return {"ok": True, "service": "cas-b3-r07-survey", "box_url": BOX_URL}


@app.get("/api/status")
def status():
    return manager.status()


@app.post("/api/session/start")
def start_session(body: dict[str, Any]):
    try:
        return manager.start_session(str(body.get("operator", ""))[:80], str(body.get("notes", ""))[:2000])
    except Exception as exc:
        raise HTTPException(409, f"Could not start survey: {exc}") from exc


@app.post("/api/session/stop-rover")
def stop_rover(body: dict[str, Any] | None = None):
    return manager.stop_rover((body or {}).get("reason", "operator_stop"))


@app.post("/api/session/enable-manual")
def enable_manual():
    try:
        return manager.enable_manual_drive()
    except Exception as exc:
        raise HTTPException(409, f"Manual drive not enabled: {exc}") from exc


@app.post("/api/drive")
def drive(body: dict[str, Any]):
    try:
        left = float(body.get("left", 0))
        right = float(body.get("right", 0))
        if not (-MAX_DRIVE <= left <= MAX_DRIVE and -MAX_DRIVE <= right <= MAX_DRIVE):
            raise ValueError(f"Manual survey command must be within +/-{MAX_DRIVE:.2f}")
        return manager.drive(left, right)
    except PermissionError as exc:
        raise HTTPException(423, str(exc)) from exc
    except (TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc)) from exc
    except Exception as exc:
        raise HTTPException(502, f"Drive request failed: {exc}") from exc


@app.post("/api/capture")
def capture(body: dict[str, Any] | None = None):
    try:
        return manager.capture(str((body or {}).get("notes", ""))[:1000])
    except Exception as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/api/annotations")
def annotate(body: dict[str, Any]):
    try:
        return manager.annotate(body)
    except Exception as exc:
        raise HTTPException(400, str(exc)) from exc


@app.post("/api/segments/action")
def segment_action(body: dict[str, Any]):
    try:
        return manager.segment_action(body)
    except Exception as exc:
        raise HTTPException(400, str(exc)) from exc


@app.post("/api/wall/decision")
def wall_decision(body: dict[str, Any]):
    try:
        return manager.wall_decision(body)
    except Exception as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/api/centreline/decision")
def centreline_decision(body: dict[str, Any]):
    try:
        return manager.centreline_decision(body)
    except Exception as exc:
        raise HTTPException(409, str(exc)) from exc


@app.post("/api/session/finalize")
def finalize():
    try:
        return manager.finalize()
    except Exception as exc:
        raise HTTPException(500, f"Finalization failed; raw session data retained: {exc}") from exc


@app.get("/api/session/download/{session_id}")
def download(session_id: str):
    try:
        path = manager.archive(session_id)
    except Exception as exc:
        raise HTTPException(404, str(exc)) from exc
    return FileResponse(path, filename=path.name, media_type="application/zip")


def check_camera_token(token: str | None):
    if not CAMERA_TOKEN or CAMERA_TOKEN == "CHANGE_ME_CAMERA_TOKEN":
        raise HTTPException(503, "Set CAS_CAMERA_TOKEN before enabling camera uploads")
    if not token or not hmac.compare_digest(token, CAMERA_TOKEN):
        raise HTTPException(401, "Camera token invalid")


@app.get("/api/camera/session")
def camera_session(x_camera_token: str | None = Header(default=None)):
    check_camera_token(x_camera_token)
    return manager.camera_session()


@app.get("/api/camera/time")
def camera_time(x_camera_token: str | None = Header(default=None)):
    check_camera_token(x_camera_token)
    return {"laptop_utc": datetime.now(timezone.utc).isoformat(), "clock_domain": "UTC"}


@app.post("/api/camera/frame")
async def camera_frame(
    request: Request,
    x_camera_token: str | None = Header(default=None),
    x_session_id: str | None = Header(default=None),
    x_frame_id: str | None = Header(default=None),
    x_frame_sequence: str | None = Header(default=None),
    x_camera_board_id: str | None = Header(default=None),
    x_camera_monotonic_ms: str | None = Header(default=None),
    x_camera_timestamp_utc: str | None = Header(default=None),
    x_capture_event_id: str | None = Header(default=None),
):
    check_camera_token(x_camera_token)
    active = manager.camera_session()
    if not active.get("active"):
        raise HTTPException(409, "No active investigation session")
    if not x_session_id or x_session_id != active.get("session_id"):
        raise HTTPException(409, "Camera session ID does not match active investigation")
    content_type = (request.headers.get("content-type") or "").lower()
    if "image/jpeg" not in content_type:
        raise HTTPException(415, "Upload must be image/jpeg")
    length = request.headers.get("content-length")
    if length and int(length) > MAX_IMAGE_BYTES:
        raise HTTPException(413, "JPEG too large")

    session_dir = manager.session["dir"]
    temp_dir = session_dir / "camera" / "frames"
    temp_dir.mkdir(parents=True, exist_ok=True)
    frame_id = re.sub(r"[^A-Za-z0-9_-]", "_", (x_frame_id or str(uuid.uuid4())))[:80]
    if not frame_id:
        frame_id = str(uuid.uuid4())
    temp_path = temp_dir / f".{frame_id}.part"
    final_path = temp_dir / f"{frame_id}.jpg"
    if final_path.exists():
        frame_id = frame_id + "_" + uuid.uuid4().hex[:8]
        temp_path = temp_dir / f".{frame_id}.part"
        final_path = temp_dir / f"{frame_id}.jpg"
    total = 0
    digest = hashlib.sha256()
    try:
        with temp_path.open("wb") as out:
            async for chunk in request.stream():
                if not chunk:
                    continue
                total += len(chunk)
                if total > MAX_IMAGE_BYTES:
                    raise HTTPException(413, "JPEG exceeded configured upload size")
                digest.update(chunk)
                out.write(chunk)
            out.flush()
            os.fsync(out.fileno())
        if total < 4:
            raise HTTPException(400, "Empty or invalid image payload")
        with temp_path.open("rb") as fp:
            if fp.read(2) != b"\xff\xd8":
                raise HTTPException(400, "JPEG SOI marker missing")
            fp.seek(-2, os.SEEK_END)
            if fp.read(2) != b"\xff\xd9":
                raise HTTPException(400, "JPEG EOI marker missing")
        try:
            with Image.open(temp_path) as im:
                im.verify()
            with Image.open(temp_path) as im:
                width, height = im.size
                if width <= 0 or height <= 0 or width > 4096 or height > 3072:
                    raise HTTPException(400, "Unexpected image dimensions")
        except UnidentifiedImageError as exc:
            raise HTTPException(400, "JPEG decoder rejected image") from exc
        temp_path.replace(final_path)
    except Exception:
        temp_path.unlink(missing_ok=True)
        raise

    received = datetime.now(timezone.utc).isoformat()
    seq = int(x_frame_sequence) if x_frame_sequence and x_frame_sequence.isdigit() else None
    cam_ms = int(x_camera_monotonic_ms) if x_camera_monotonic_ms and x_camera_monotonic_ms.isdigit() else None
    meta = {
        "frame_id": frame_id, "frame_sequence": seq, "camera_board_id": x_camera_board_id,
        "camera_monotonic_ms": cam_ms, "camera_timestamp_utc": x_camera_timestamp_utc,
        "laptop_received_at": received, "width": width, "height": height, "bytes": total,
        "sha256": digest.hexdigest(), "capture_event_id": x_capture_event_id,
    }
    try:
        manager.camera_saved(meta, final_path)
    except Exception as exc:
        final_path.unlink(missing_ok=True)
        manager.camera_errors += 1
        raise HTTPException(500, f"Frame could not be indexed: {exc}") from exc

    # Observational classical CV only; it can never command movement.
    try:
        image_bytes = final_path.read_bytes()
        vision = await asyncio.to_thread(detect_tactile_strip_jpeg, image_bytes)
        manager.set_vision(frame_id, vision)
        meta["vision"] = vision
    except Exception as exc:
        manager.set_vision(frame_id, {"available": False, "valid": False, "reason": f"{type(exc).__name__}: {exc}"})
    return {"ok": True, **meta}


@app.get("/api/camera/latest")
def camera_latest():
    with manager.lock:
        s = manager.session
        frame = s.get("latest_frame_id") if s else None
        if not s or not frame:
            raise HTTPException(404, "No camera frame received")
        path = s["dir"] / "camera" / "frames" / f"{frame}.jpg"
        if not path.exists():
            raise HTTPException(404, "Latest camera frame missing from disk")
        return FileResponse(path, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/camera/commands")
def camera_commands(x_camera_token: str | None = Header(default=None)):
    check_camera_token(x_camera_token)
    status = manager.camera_session()
    return {"active": status["active"], "session_id": status["session_id"], "capture": status["capture"]}


@app.get("/api/system/config")
def system_config():
    return {
        "main_box_url": BOX_URL,
        "camera_transport": "ESP32-S3 joins CAS-Box-07 and posts JPEG directly to this laptop service; main ESP32 does not relay image bytes.",
        "camera_token_required": True,
        "camera_board_pin_map_available": False,
        "manual_drive_limit": MAX_DRIVE,
        "physical_estop_available_in_repo": False,
        "wall_following_motion_enabled": False,
        "notes": [
            "A laptop-side stop requires Wi-Fi to the box. Use a physical hardware stop if fitted; none is wired in this repository.",
            "Main ESP32 RoverLink deadman remains the last software layer if commands expire.",
            "Camera board GPIO/pin mapping must be filled from the exact ESP32-S3 camera hardware before camera firmware is compiled.",
        ],
    }
