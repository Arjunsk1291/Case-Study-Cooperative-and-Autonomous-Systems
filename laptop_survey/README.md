# CAS R04 — Offline survey log packager

This folder is used **after** the field survey. It is not a live robot collector.

The current app.py:
- does not connect to CAS-Box-04;
- does not poll /values or camera endpoints;
- does not expose motor-drive or STOP commands;
- does not process images or build an indoor map;
- packages the phone-exported sensor/route log and camera microSD JPEGs into one ZIP, with SHA-256 checksums.

## Run

On the laptop, after copying the survey files from the phone and camera archive:

    cd laptop_survey
    python3 -m venv .venv
    source .venv/bin/activate
    pip install -r requirements.txt
    uvicorn app:app --host 127.0.0.1 --port 8000

Open http://127.0.0.1:8000. Select the main ESP32 dashboard's JSON/CSV export, the camera manifest.csv, and all relevant JPEG files. The maximum combined upload is 500 MB. The app creates a checksummed ZIP and offers it for download.

Attach that ZIP to the analysis conversation. The original files are preserved; this packager does not infer coordinates, maps or motor commands.

## Return path

After offline review, the analysis result can be a compact JSON document using schema CAS-R04-NAV-CMD-1, mode route_pending, and up to 12 waypoint records. Paste that into the phone dashboard's **Offline navigation instructions** panel while the phone is connected to CAS-Box-04.

The main ESP32 stores the compact plan draft in RAM. It explicitly reports execution_enabled=false. There is no validated indoor localization/odometry/waypoint executor yet, so this import is not a movement command.

## Legacy files

manager.py and its tests are retained for historical reference to the earlier laptop-owned collector prototype. The current app.py does not import or start that collector. Do not run old manager code as a live R04 robot-control service.
