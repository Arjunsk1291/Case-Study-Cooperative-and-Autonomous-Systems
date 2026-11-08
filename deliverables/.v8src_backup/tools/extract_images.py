#!/usr/bin/env python3
"""extract_images.py - img_NNN.bin (camera snapshot pictures saved by cas_box_mission) -> JPEG files + index.

  python3 extract_images.py img_001.bin        # -> folder img_001/ with snap_<n>.jpg (upright) + index.csv
Per picture in the file: "IMG1", record index (u32), ms (u32), snapId (u16), strip ok (u8), rows (u8),
offset px x10 (i16), angle deg x10 (i16), JPEG length (u16), then the JPEG.
The record index matches the row in mis_NNN.csv (record k = row k), so every picture can be put next to the drive data.
"""
import csv, os, struct, sys

HDR = "<4sIIHBBhhH"
HSIZE = struct.calcsize(HDR)


def extract(path):
    data = open(path, "rb").read()
    out = path.rsplit(".", 1)[0]
    os.makedirs(out, exist_ok=True)
    rows, i, n = [], 0, 0
    while i + HSIZE <= len(data):
        m, rec, ms, sid, ok, nrows, off, ang, ln = struct.unpack(HDR, data[i:i + HSIZE])
        if m != b"IMG1" or i + HSIZE + ln > len(data):
            print(f"stop at byte {i}: damaged or cut record"); break
        jpg = data[i + HSIZE:i + HSIZE + ln]
        name = f"snap_{n:03d}_rec{rec:05d}.jpg"
        try:                                   # save upright (the camera is mounted 90 deg rotated)
            import cv2, numpy as np
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
            cv2.imwrite(os.path.join(out, name), cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE))
        except Exception:
            open(os.path.join(out, name), "wb").write(jpg)
        rows.append(dict(file=name, record=rec, ms=ms, snap_id=sid, strip_ok=ok, rows=nrows, offset_px=off / 10, angle_deg=ang / 10, bytes=ln))
        i += HSIZE + ln; n += 1
    with open(os.path.join(out, "index.csv"), "w", newline="") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print(f"{path}: {n} pictures -> {out}/")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        extract(p)
