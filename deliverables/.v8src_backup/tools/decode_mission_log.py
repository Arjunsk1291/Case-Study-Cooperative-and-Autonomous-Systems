#!/usr/bin/env python3
"""decode_mission_log.py - mis_NNN.bin (cas_box_mission, log v4) -> CSV + event timeline.

  python3 decode_mission_log.py mis_001.bin            # writes mis_001.csv and prints the events
The record layout must match MissionLog::Rec (the ESP32 build checks the size: 249 bytes).
Firmware v3 (no camera, 07.11. evening) re-uses the camera fields: turn_tactic, turn_cycles, turn_shifts,
speed_boost, motion_m_s, ref_type (v5: open-sea heading reference). In leg 2+ crab_deg = leg-1 drift (logged only); in the open sea wall_d_m = corridor offset (m), wall_ang_deg = side term (deg). Use --v2 for logs of the camera version.
"""
import csv, struct, sys

FMT = "<IiiHBBhhhBBBBbbbbBH5H90HhhhHhHHBBhhHHBB"
SIZE = struct.calcsize(FMT)
STATES = ["ready", "leg1_follow", "t1_ask", "t1_turn", "t1_settle", "l2_look", "l2_ask", "l2_board", "l2_go", "hold",
          "end_ask", "manual", "paused", "done", "l3_follow", "l4_follow"]
EVENTS = {0: "", 1: "start", 2: "pillar", 3: "t1_ask", 4: "op_turn", 5: "op_on1m", 6: "manual_on", 7: "manual_off",
          8: "turn_done", 9: "look", 10: "align", 11: "go_leg2", 12: "check_ok", 13: "check_fail", 14: "hold",
          15: "hold_end", 16: "end_wall", 17: "pause", 18: "resume", 19: "mark", 20: "leg2_max", 21: "failsafe",
          22: "right_open", 23: "wall_lost", 24: "wall_found", 25: "snap_ok", 26: "snap_fail", 27: "done",
          28: "turn_stall", 29: "image", 30: "leg3_start", 31: "wall_end", 32: "turn_pillar", 33: "aim_done",
          34: "realign", 35: "board_seen", 36: "board_lock", 37: "end_board", 38: "t3_turn", 39: "leg4_start",
          40: "front_is_wall", 41: "ref_rear_wall", 42: "ref_front_wall", 43: "ref_board",
          44: "ref_left_row", 45: "ref_left_building", 46: "leg4_wall",
          47: "pause_stop_button", 48: "pause_phone_lost", 49: "pause_rover_lost", 50: "pause_tilt", 51: "pause_turn_failed",
          52: "pause_blocked_20s", 53: "pause_heading_failsafe",
          54: "lidar_ghost_ignored", 55: "climb", 56: "record_manual_drive", 57: "rover_link_wait", 58: "retry"}


V2 = False
TACTICS = ["pivot", "rest", "swing", "arc", "shift"]


def decode(path):
    data = open(path, "rb").read()
    magic, ver, npts, rsize, period, start = struct.unpack("<6sBBHHI", data[:16])
    if magic != b"CASLOG" or ver != 4:
        raise SystemExit(f"{path}: not a v4 mission log (magic {magic}, version {ver})")
    if rsize != SIZE:
        raise SystemExit(f"{path}: record size {rsize} != decoder {SIZE} - decoder and firmware differ")
    n = (len(data) - 16) // rsize
    rows = []
    for k in range(n):
        v = struct.unpack(FMT, data[16 + k * rsize: 16 + (k + 1) * rsize])
        i = 0
        def take(c):
            nonlocal i
            out = v[i:i + c]; i += c
            return out if c > 1 else out[0]
        ms = take(1); lat, lon = take(2); hdop = take(1); sats = take(1); flags = take(1)
        yaw, pitch, roll = take(3); state, leg, ev, pillars = take(4); cl, cr, ol, orr = take(4)
        rtt = take(1); batt = take(1); us = take(5); lid = take(90)
        tgt, err, crab = take(3); wd = take(1); wa = take(1); dist = take(1); front = take(1)
        rows_, conf = take(2); coff, cang = take(2); ssent, sok = take(2); loopx4, ds = take(2)
        r = dict(t_s=round((ms - start) / 1000.0, 2), state=STATES[state] if state < len(STATES) else state, leg=leg,
                 event=EVENTS.get(ev, ev), pillars=pillars,
                 lat=lat / 1e7, lon=lon / 1e7, hdop=hdop / 100, near_inten=round(hdop / 100), sats=sats, gps_fix=flags & 1,
                 imu_ok=(flags >> 1) & 1, lidar_ok=(flags >> 2) & 1, cam_paired=(flags >> 3) & 1, manual=(flags >> 4) & 1,
                 hold=(flags >> 5) & 1, slope=(flags >> 6) & 1, snap_strip=(flags >> 7) & 1,
                 yaw=yaw / 100, pitch=pitch / 100, roll=roll / 100, cmd_l=cl / 200, cmd_r=cr / 200, out_l=ol / 200, out_r=orr / 200,
                 rtt_ms=rtt * 10, battery_v=batt / 100,
                 us_front=us[0], us_left=us[1], us_right=us[2], us_back=us[3], us_front_angled=us[4],
                 target_deg=tgt / 100, heading_err_deg=err / 100, crab_deg=crab / 100, wall_d_m=wd / 100, wall_ang_deg=wa / 10,
                 leg_dist_m=dist / 100, front_m=front / 100, loop_max_ms=loopx4 * 4, ds_status=ds)
        if V2:
            r.update(cam_rows=rows_, cam_conf=conf, cam_off_px=coff / 10, cam_ang_deg=cang / 10, snaps_sent=ssent, snaps_ok=sok)
        else:
            r.update(turn_tactic=TACTICS[rows_] if rows_ < len(TACTICS) else rows_, turn_cycles=conf, turn_shifts=ssent,
                     ref_type=["gyro", "rear_wall", "front_wall", "board", "left_row", "left_building"][sok] if sok < 6 else sok,
                     speed_boost=coff / 1000, motion_m_s=cang / 1000)
        for j in range(npts):
            r[f"lidar_{j * 4:03d}"] = lid[j]
        rows.append(r)
    return rows


if __name__ == "__main__":
    V2 = "--v2" in sys.argv
    for p in [a for a in sys.argv[1:] if not a.startswith("--")]:
        rows = decode(p)
        out = p.rsplit(".", 1)[0] + ".csv"
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        print(f"{p}: {len(rows)} records ({rows[-1]['t_s']:.0f} s) -> {out}")
        for r in rows:
            if r["event"]:
                print(f"  {r['t_s']:7.1f} s  {r['event']:11s} state {r['state']:12s} leg {r['leg']} dist {r['leg_dist_m']:5.1f} m "
                      f"pillars {r['pillars']} wall {r['wall_d_m']:.2f} crab {r['crab_deg']:+.1f} front {r['front_m']:.2f} "
                      + (f"cam {r['cam_rows']} rows off {r['cam_off_px']:+.0f} ang {r['cam_ang_deg']:+.1f}" if V2 else
                         f"yaw {r['yaw']:.1f} tactic {r['turn_tactic']} boost {r['speed_boost']:.2f}"))
