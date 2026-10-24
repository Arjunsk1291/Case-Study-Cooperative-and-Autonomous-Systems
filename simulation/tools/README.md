# R07 simulation tools

## Interactive 2D campus route simulator
Install and run on Ubuntu:
```bash
cd simulation/tools
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 r07_campus_sim.py
```

Controls: **A** = plan and run Block B → Block H, **P** = pause with zero command, **Space** = immediate stop, **R** = reset, **WASD / arrows** = manual motion, **+/-** = zoom, mouse wheel = zoom, **Esc** = quit.

Run the offline planner test (does not open a GUI):
```bash
python3 r07_campus_sim.py --self-test
```

The simulator uses a 0.30 × 0.20 m rover footprint, a simulated 90-bin / 4° LiDAR, five ultrasonic rays in the firmware's order, and a differential-drive command model. The CAS Box enclosure is shown as a separate top-mounted blue rectangle; its dimensions and several sensor mounts are placeholders pending physical measurements. The simulation produces idealized ray distances and does not run the actual ESP32 firmware.

The terrain JSON is traced from the official TH Deggendorf building diagram and is intentionally labelled **SCHEMATIC_APPROXIMATION_NOT_SURVEYED**. Its metric scale, footprints, entrances and walkways are estimates. Edit `../terrain/dit_deggendorf_b_to_h.json` to revise the layout.

## LiDAR signature plot
```bash
python3 scan_to_polar_map.py --label END_SCAN_entrance
```
This writes a CSV and polar PNG for the last recorded R07 scan signature. To plot another waypoint use e.g. `--label "pillar 1"`. To use a downloaded investigation JSON:
```bash
python3 scan_to_polar_map.py /path/to/investigation.json --label "pillar 1" --out ./plots
```
A polar scan plot is not a 2D occupancy map and does not recover the rover's x/y pose.

## Official campus references
- [THD campus map / building plan](https://www.th-deg.de/Hochschule/anfahrt)
- [Map Deggendorf and building index (PDF)](https://www.th-deg.de/Fakult%C3%A4ten/agw/Guide-LSI.pdf)

The Block B → H route is a starting hypothesis based on the campus diagram, not verified pedestrian routing. Before deployment, confirm actual entrances, accessible paths, ramps/steps, road crossings, access restrictions and dimensions on site.

The simulator also displays an approximate GPS latitude/longitude derived from the local terrain origin. This is a demonstration coordinate transform only; it is not a surveyed georeference, and GPS should not be used to validate the candidate B-to-H route.
