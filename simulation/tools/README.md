# R07 campus route simulator (no hardware needed)

## Install / run on Ubuntu
```bash
cd simulation/tools
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 r07_campus_sim.py
```
The simulator reads `../terrain/dit_deggendorf_b_to_h.json`. Press **A** to plan and run B-to-H, **R** to reset, **WASD / arrows** for manual drive, **Space** to stop, **+/-** to zoom. The screen shows the approximate TH Deggendorf block layout, planned path, robot footprint, 90-bin LiDAR returns and five ultrasonic rays. Finishing the route commands zero motion and latches FINISHED until reset.

## Scope / uncertainty
The map is traced from the official campus diagram (Guide-LSI.pdf, page 11) as a schematic. The official drawing is not a scaled survey, so the coordinate values and route are placeholders that must be adjusted. Known R07 rover footprint (30 x 20 cm) is represented; box enclosure dimensions and sensor mounting locations are configurable estimates. The sim models idealized sensing/motion and cannot prove hardware performance.

Official source: https://www.th-deg.de/Fakult%C3%A4ten/agw/Guide-LSI.pdf
