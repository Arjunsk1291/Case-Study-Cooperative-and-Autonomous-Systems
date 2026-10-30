from __future__ import annotations

"""Offline R04 log packager. This service never connects to or controls the robot."""

import hashlib
import json
import os
import re
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path

from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
DATA_ROOT = Path(os.environ.get("CAS_R04_OFFLINE_DATA", str(Path.home() / "CAS_R04_Offline_Logs"))).resolve()
SESSIONS = DATA_ROOT / "sessions"
ARCHIVES = DATA_ROOT / "archives"
MAX_FILE_BYTES = 100 * 1024 * 1024
MAX_TOTAL_BYTES = 500 * 1024 * 1024
ALLOWED_SUFFIXES = {".json", ".csv", ".jpg", ".jpeg", ".png", ".txt", ".log", ".zip"}

SESSIONS.mkdir(parents=True, exist_ok=True)
ARCHIVES.mkdir(parents=True, exist_ok=True)

app = FastAPI(title="CAS R04 Offline Log Packager", version="2.0.0")
app.mount("/static", StaticFiles(directory=STATIC), name="static")


def safe_name(name: str) -> str:
    base = Path(name or "upload.bin").name
    base = re.sub(r"[^A-Za-z0-9_.-]+", "_", base).strip("._")
    return (base or "upload.bin")[:120]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/health")
def health():
    return {
        "ok": True,
        "service": "offline-log-packager",
        "robot_connection": False,
        "robot_control": False,
        "max_total_bytes": MAX_TOTAL_BYTES,
    }


@app.post("/api/package")
async def package_logs(
    files: list[UploadFile] = File(...),
    session_label: str = Form("R04 survey"),
):
    if not files:
        raise HTTPException(400, "Select at least one log or image file.")

    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:8]
    session_dir = SESSIONS / session_id
    originals = session_dir / "originals"
    originals.mkdir(parents=True, exist_ok=False)

    entries = []
    total_bytes = 0
    try:
        for index, upload in enumerate(files, start=1):
            name = safe_name(upload.filename or f"file_{index}")
            suffix = Path(name).suffix.lower()
            if suffix not in ALLOWED_SUFFIXES:
                raise HTTPException(415, f"Unsupported file type: {name}")
            payload = await upload.read(MAX_FILE_BYTES + 1)
            if not payload:
                raise HTTPException(400, f"Empty file: {name}")
            if len(payload) > MAX_FILE_BYTES:
                raise HTTPException(413, f"File exceeds 100 MB: {name}")
            total_bytes += len(payload)
            if total_bytes > MAX_TOTAL_BYTES:
                raise HTTPException(413, "Combined upload exceeds 500 MB.")

            # Prefix duplicate names so no supplied file silently overwrites another.
            stored_name = f"{index:03d}_{name}"
            destination = originals / stored_name
            destination.write_bytes(payload)
            item = {
                "original_name": upload.filename or name,
                "stored_name": stored_name,
                "bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "extension": suffix,
            }
            if suffix == ".json":
                try:
                    parsed = json.loads(payload.decode("utf-8"))
                    item["json_root_type"] = type(parsed).__name__
                    if isinstance(parsed, dict):
                        item["json_keys"] = list(parsed.keys())[:40]
                except (UnicodeDecodeError, json.JSONDecodeError):
                    item["json_parseable"] = False
            entries.append(item)
            await upload.close()

        manifest = {
            "schema": "CAS-R04-OFFLINE-UPLOAD-1",
            "session_id": session_id,
            "label": session_label[:100],
            "created_at": datetime.now(timezone.utc).isoformat(),
            "file_count": len(entries),
            "total_bytes": total_bytes,
            "files": entries,
            "processing_note": (
                "Files are preserved as supplied. This packager does not infer maps, "
                "coordinates, motion commands or route safety."
            ),
        }
        manifest_path = session_dir / "upload_manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

        archive_path = ARCHIVES / f"CAS_R04_Offline_Log_{session_id}.zip"
        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_STORED) as archive:
            archive.write(manifest_path, "upload_manifest.json")
            for path in sorted(originals.iterdir()):
                archive.write(path, f"originals/{path.name}")

        with zipfile.ZipFile(archive_path, "r") as archive:
            bad_member = archive.testzip()
            if bad_member:
                raise RuntimeError(f"ZIP verification failed for {bad_member}")

        return {
            "ok": True,
            "session_id": session_id,
            "file_count": len(entries),
            "total_bytes": total_bytes,
            "archive_bytes": archive_path.stat().st_size,
            "sha256": sha256_file(archive_path),
            "download_url": f"/api/download/{session_id}",
            "manifest": manifest,
        }
    except Exception:
        # Retain only completed bundles; remove incomplete upload directories.
        if session_dir.exists() and not (ARCHIVES / f"CAS_R04_Offline_Log_{session_id}.zip").exists():
            import shutil
            shutil.rmtree(session_dir, ignore_errors=True)
        raise


@app.get("/api/download/{session_id}")
def download_package(session_id: str):
    if not re.fullmatch(r"\d{8}T\d{6}_[a-f0-9]{8}", session_id):
        raise HTTPException(400, "Invalid session ID.")
    path = ARCHIVES / f"CAS_R04_Offline_Log_{session_id}.zip"
    if not path.is_file():
        raise HTTPException(404, "Offline log package not found.")
    return FileResponse(path, filename=path.name, media_type="application/zip")
