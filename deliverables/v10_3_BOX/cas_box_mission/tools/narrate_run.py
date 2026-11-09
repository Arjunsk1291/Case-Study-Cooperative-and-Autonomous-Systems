#!/usr/bin/env python3
"""narrate_run.py - "what was the rover thinking": turns a mission log into a plain-language story.

  python3 narrate_run.py mis_001.bin            # story on the screen
  python3 narrate_run.py mis_001.bin --md out.md

For every phase (leg 1, turn, open sea, leg 3, T3, leg 4) it says what the rover used as reference, what it
measured, what it decided and why it stopped. Problems are flagged automatically (failsafe loops, target jumps,
heading far from target, references that disagree, pauses, take-overs).
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode_mission_log as D
from fwmirror import w180

PHASE = {"leg1_follow": "Leg 1 (B facade, wall on the left)", "t1_turn": "Turning", "t1_settle": "Checking the turn",
         "l2_board": "Open-sea start (board)", "l2_go": "Open sea", "hold": "Waiting (something in front)",
         "l3_follow": "Leg 3 (arcade, wall on the left)", "l4_follow": "Leg 4 (after T3, wall on the left)",
         "manual": "MANUAL (you drove)", "paused": "PAUSED", "end_ask": "Stopped at the goal", "done": "Finished"}
REF = {"gyro": "gyro only", "rear_wall": "the B wall behind", "front_wall": "the end wall ahead", "board": "the board",
       "left_row": "the tree/post row on the left", "left_building": "the building front on the left"}
WHY = {"pillar": "saw a pillar on the right", "turn_pillar": "found the turn landmark (last pillar) -> turn right",
       "wall_lost": "the wall on the left disappeared (gap / end)", "wall_found": "found a wall on the left again",
       "end_wall": "end wall ahead at the stop distance -> turn right", "failsafe": "HEADING FAILSAFE (target too far from the heading for 2 s)",
       "realign": "stopped to turn parallel again", "hold": "something in front -> waiting", "t3_turn": "arcade wall ended -> T3 left turn",
       "ref_rear_wall": "started using the B wall behind", "ref_left_row": "found the tree/post row on the left",
       "ref_front_wall": "started using the end wall ahead", "leg4_wall": "found the wall after T3", "manual_on": "you took over",
       "manual_off": "you gave it back", "pause": "paused (STOP / phone / failsafe)", "resume": "resumed", "turn_stall": "turn stalled -> tried the next way",
       "wall_end": "the wall ended -> stop", "turn_done": "turn finished",
       "pause_stop_button": "  -> because: STOP pressed", "pause_phone_lost": "  -> because: phone page lost (screen off / other app)",
       "pause_rover_lost": "  -> because: rover link lost", "pause_tilt": "  -> because: tilt", "pause_turn_failed": "  -> because: turn failed 2 min",
       "pause_blocked_20s": "  -> because: blocked 20 s", "pause_heading_failsafe": "  -> because: heading failsafe / IMU",
       "lidar_ghost_ignored": "ignored a single near LiDAR point (daylight ghost, not confirmed by beams/ultrasonic)",
       "climb": "CLIMB button: short 0.35 push", "record_manual_drive": "RECORDING a manual drive", "rover_link_wait": "rover link lost - waiting",
       "retry": "tried again by itself (blocked / turn)"}


def story(R):
    out = []; seg = None; pend = []
    def flush():
        if not seg: return
        a, b = seg["rows"][0], seg["rows"][-1]
        st = a["state"]; dur = b["t_s"] - a["t_s"]
        line = f"{a['t_s']:6.1f}-{b['t_s']:6.1f} s  {PHASE.get(st, st)} ({dur:.0f} s"
        if st in ("leg1_follow", "l3_follow", "l4_follow", "l2_go"):
            line += f", {a['leg_dist_m']:.1f} -> {b['leg_dist_m']:.1f} m"
        out.append(line + ")")
        rows = seg["rows"]
        if st in ("leg1_follow", "l3_follow", "l4_follow"):
            w = [r["wall_d_m"] for r in rows if r["wall_d_m"] > 0]
            if w: out.append(f"      wall on the left: {min(w):.2f}-{max(w):.2f} m (median {sorted(w)[len(w)//2]:.2f})")
            else: out.append("      no wall seen on the left - drove on the gyro heading")
        if st == "l2_go":
            cnt = {}
            for r in rows: cnt[r["ref_type"]] = cnt.get(r["ref_type"], 0) + 1
            n = len(rows)
            out.append("      references: " + ", ".join(f"{REF.get(k, k)} {100*v//n}%" for k, v in sorted(cnt.items(), key=lambda kv: -kv[1])))
            ro = [r["wall_d_m"] for r in rows if r["ref_type"] == "left_row" and r["wall_d_m"] > 0]
            if ro: out.append(f"      tree/post row distance {min(ro):.2f}-{max(ro):.2f} m (median {sorted(ro)[len(ro)//2]:.2f}) -> "
                              "it thought it was on the strip when this was near the setting")
            tg = [r["target_deg"] for r in rows]
            out.append(f"      wanted heading moved {min(tg):.1f} .. {max(tg):.1f} deg; heading error median "
                       f"{sorted(abs(r['heading_err_deg']) for r in rows)[n//2]:.1f} deg")
        for t, ev in pend: out.append(f"      {t:6.1f} s  {WHY.get(ev, ev)}")
    for r in R:
        st = r["state"]
        if seg is None or st != seg["st"]:
            flush(); seg = {"st": st, "rows": []}; pend = []
        seg["rows"].append(r)
        if r["event"] and r["event"] in WHY: pend.append((r["t_s"], r["event"]))
    flush()
    return out


def problems(R):
    p = []
    fs = [r["t_s"] for r in R if r["event"] == "failsafe"]
    if len(fs) >= 2: p.append(f"{len(fs)} heading failsafes ({', '.join(f'{t:.0f}' for t in fs[:6])} s): target and heading disagreed for 2 s")
    for a, b in zip(R, R[1:]):
        if a["state"] == b["state"] and a["state"] in ("l2_go", "l3_follow", "l4_follow", "leg1_follow") and abs(w180(b["target_deg"] - a["target_deg"])) > 6:
            p.append(f"{b['t_s']:.1f} s: wanted heading jumped {w180(b['target_deg'] - a['target_deg']):+.1f} deg in one record ({b['state']})")
    big = [r for r in R if r["state"] in ("l2_go", "l3_follow", "l4_follow", "leg1_follow") and abs(r["heading_err_deg"]) > 12]
    if big: p.append(f"heading more than 12 deg off the target for {len(big)/4:.1f} s in total (first at {big[0]['t_s']:.1f} s)")
    near = sum(1 for r in R if any(0 < r["lidar_%03d" % (i * 4)] < 35 for i in range(90)))
    if near > 0.05 * len(R): p.append(f"LiDAR points closer than 35 cm in {100*near/len(R):.0f} % of the scans (daylight noise; v8 ignores them)")
    man = [r["t_s"] for r in R if r["event"] == "manual_on"]
    if man: p.append(f"you took over {len(man)}x ({', '.join(f'{t:.0f}' for t in man)} s)")
    return p


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("log"); ap.add_argument("--md")
    a = ap.parse_args()
    R = D.decode(a.log)
    lines = [f"# What the rover thought - {os.path.basename(a.log)} ({R[-1]['t_s']:.0f} s)", ""] + story(R)
    pr = problems(R)
    lines += ["", "## Problems found" if pr else "## No problems found"] + [f"- {x}" for x in pr]
    txt = "\n".join(lines)
    print(txt)
    if a.md: open(a.md, "w").write(txt + "\n")


if __name__ == "__main__":
    main()
