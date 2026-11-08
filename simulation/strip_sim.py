"""strip_sim.py - "position once, then drive straight" model of kit 04 + policy search (2026-11-07).

Model (all numbers from our measurements unless marked GUESS):
  heading hold   Kethan PD on the gyro: heading follows its target with ~1 s time constant, max turn rate
                 34.4 deg/s per unit wheel difference x correction limit 0.08 (v=0.20) = 2.8 deg/s   (log3)
  drift          residual after trim: the hold absorbs it (small steady error, learned away)
  CRAB           the rover slides sideways at a constant angle while POINTING straight: the gyro cannot see it.
                 outdoor -5.6 .. -7.6 deg (logs 026-028), indoor GUESS 0 +- 1.5 deg
  gyro           bias drift (deg/s) accumulates into the heading the hold believes
  camera snap    on request, rover standing or rolling; p_detect, false alarms (random strip), offset/angle
                 noise, unknown pixel scale (px per m), mounting angle error, link failure, camera dead
  pivot          align turn error ~ N(0, 2 deg)
Policy (what the box does):
  ALIGN   take n_align snaps (median of the valid ones); if any: pivot by -angle; optional verify snaps
  MERGE   steer towards the strip: target = -atan(offset / D_merge); back to straight after D_merge
  GO      hold the heading; every check_every metres take n_check snaps; if valid and plausible,
          heading += clip(-(gA*angle + gO*atan(offset/D_merge)), +-max_corr). NEVER stop for the camera.
Vectorised over episodes (numpy). Run:  python3 strip_sim.py
"""
import numpy as np

DT = 0.2
NOISE = 1.0   # scales mounting / pivot / angle noise (1 = realistic, 0 = perfect) - for debugging

SCEN = {
    # name: strip length to the wall (m), start offset (m), start angle (deg), crab mean/sd (deg),
    #       gyro bias sd (deg/s), p_detect, p_false, tolerance on the strip at the end (m)
    "indoor_2m":   dict(L=2.0,  y0=0.15, a0=25, crab=(0.0, 1.5),  gb=0.02, pdet=0.90, pfa=0.05, tol=0.10),
    "plaza1_36m":  dict(L=36.0, y0=0.30, a0=20, crab=(-6.5, 1.5), gb=0.03, pdet=0.80, pfa=0.10, tol=0.50),
    "grooved_24m": dict(L=24.0, y0=0.30, a0=20, crab=(-6.5, 1.5), gb=0.03, pdet=0.20, pfa=0.10, tol=0.50),
}


def snap(rng, mask, y, psi, P, S):
    """One camera snapshot for the episodes in mask. Returns (valid, ang_meas_deg, off_meas_m)."""
    n = y.size
    ok = mask & ~P["cam_dead"] & (rng.random(n) > P["p_link"])
    det = ok & (rng.random(n) < S["pdet"])
    fa = ok & ~det & (rng.random(n) < S["pfa"])
    look = P["look"]
    off_true_m = y + look * np.sin(np.radians(psi))                       # strip to the RIGHT = +
    off_px = P["scale"] * off_true_m + rng.normal(0, 3.0, n)              # 3 px noise
    ang = psi + P["mount"] + rng.normal(0, 2.0 * NOISE + 1e-9, n)          # 2 deg noise
    ang = np.where(fa, rng.uniform(-30, 30, n), ang)                       # false alarm = garbage
    off_px = np.where(fa, rng.uniform(-60, 60, n), off_px)
    off_px = np.where(P["flip"], -off_px, off_px)                          # sign error (stress test)
    ang = np.where(P["flip"], -ang, ang)
    valid = (det | fa) & (np.abs(off_px) < 58)                             # strip must be inside the image
    off_m = off_px / P["scale_assumed"]
    return valid, ang, off_m


def multi_snap(rng, mask, y, psi, P, S, k):
    """k snapshots, median of the valid ones (robust against one false alarm)."""
    A = np.full((k, y.size), np.nan); O = np.full((k, y.size), np.nan)
    for i in range(k):
        v, a, o = snap(rng, mask, y, psi, P, S)
        A[i] = np.where(v, a, np.nan); O[i] = np.where(v, o, np.nan)
    nval = np.sum(~np.isnan(A), 0)
    import warnings
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore")
        a = np.nanmedian(A, 0); o = np.nanmedian(O, 0)
    need = 1 if k < 3 else 2                                               # with 3+ snaps: 2 must agree
    return mask & (nval >= need), np.nan_to_num(a), np.nan_to_num(o), nval


def episodes(pol, scen, n, rng, stress=None):
    S = SCEN[scen]
    P = dict(
        scale=rng.uniform(150, 600, n),                                    # true px per m (unknown)
        look=rng.uniform(0.25, 0.5, n),                                    # detector rows look this far ahead (GUESS)
        mount=rng.normal(0, 2.0 * NOISE, n),                               # camera angle mounting error
        p_link=0.05, cam_dead=np.zeros(n, bool), flip=np.zeros(n, bool),
    )
    P["scale_assumed"] = P["scale"] * rng.uniform(0.7, 1.4, n)             # our scale estimate is +-30 % off
    if stress == "camera_dead": P["cam_dead"][:] = True
    if stress == "link_50pct": P["p_link"] = 0.5
    if stress == "sign_flip": P["flip"][:] = True
    if stress == "bad_scale": P["scale_assumed"] = P["scale"] * rng.choice([0.4, 2.5], n)
    if stress == "mount_8deg": P["mount"] = rng.choice([-8.0, 8.0], n)
    if stress == "mount_cal": P["mount"] = P["mount"] * 0.25                # after a "this is straight" calibration
    crab = rng.normal(*S["crab"], n)
    if stress == "crab_x2": crab = crab * 2
    gbias = rng.normal(0, S["gb"], n)
    y = rng.uniform(-S["y0"], S["y0"], n)
    psi = rng.uniform(-S["a0"], S["a0"], n)                                # true heading vs strip (+ left)
    gerr = np.zeros(n)                                                     # gyro heading error
    snaps = np.zeros(n)

    # ---- ALIGN (standing)
    ok, a, o, nv = multi_snap(rng, np.ones(n, bool), y, psi, P, S, pol["n_align"]); snaps += pol["n_align"]
    psi = np.where(ok, psi - a + rng.normal(0, 2.0 * NOISE + 1e-9, n), psi)               # pivot by the measured angle
    if pol["verify"]:
        ok2, a2, o2, _ = multi_snap(rng, np.ones(n, bool), y, psi, P, S, pol["n_align"]); snaps += pol["n_align"]
        psi = np.where(ok2 & (np.abs(a2) > 3), psi - a2 + rng.normal(0, 2.0 * NOISE + 1e-9, n), psi)
        ok = ok | ok2; o = np.where(ok2, o2, o)
    # target heading in the TRUE frame = current heading (the hold keeps what the gyro believes)
    tgt = psi.copy()
    D = pol["D_merge"]
    # CRAB: start from the measured prior (outdoor -6.5 deg from logs 026-028, indoor 0) times use_prior,
    # then learn from two camera checks. Wanted track = 0  ->  heading = -crab_est.
    crab_est = np.full(n, S["crab"][0] * pol["use_prior"])
    psi = psi - crab_est                                                    # the align turn already aims for track 0
    last_x = np.full(n, np.nan); last_o = np.full(n, np.nan); last_a = np.full(n, np.nan)
    merge_left = np.where(ok, D, 0.0)                                      # metres of merge left
    off_est = np.where(ok, o, 0.0)
    straight = psi.copy()                                                  # heading to return to after merging
    tgt = np.where(ok, straight - np.degrees(np.arctan2(off_est, D)), tgt)

    # ---- GO
    x = np.zeros(n); t = 0.0
    v = 0.20 * 1.03
    next_chk = np.full(n, pol["check_every"])
    running = np.ones(n, bool)
    ymax = np.abs(y)
    while running.any() and t < 400:
        t += DT
        # heading hold: true heading follows (target - gyro error), first order, rate limited
        want = (tgt - gerr) - psi
        rate = np.clip(want / 1.0, -2.8, 2.8)
        psi = np.where(running, psi + rate * DT, psi)
        gerr = gerr + gbias * DT
        track = np.radians(psi + crab)
        x = np.where(running, x + v * np.cos(track) * DT, x)
        y = np.where(running, y + v * np.sin(track) * DT, y)
        ymax = np.maximum(ymax, np.abs(y))
        # merge finished -> back to the straight heading
        done_m = running & (merge_left > 0)
        merge_left = np.where(done_m, merge_left - v * DT, merge_left)
        back = done_m & (merge_left <= 0)
        tgt = np.where(back, straight, tgt)
        # periodic check
        chk = running & (x >= next_chk)
        if chk.any():
            okc, ac, oc, _ = multi_snap(rng, chk, y, psi, P, S, pol["n_check"]); snaps += chk * pol["n_check"]
            plaus = okc & (np.abs(ac) < pol["ang_tol"]) & (np.abs(oc) < pol["off_tol"])
            corr = -(pol["gA"] * ac + pol["gO"] * np.degrees(np.arctan2(oc, D)))
            corr = np.clip(corr, -pol["max_corr"], pol["max_corr"])
            straight = np.where(plaus, straight + corr, straight)
            # crab learning: lateral drift between two good checks vs the body angle the camera saw
            have = plaus & ~np.isnan(last_x) & (x - np.nan_to_num(last_x) > 1.0)
            if have.any() and pol["crab_k"] > 0:
                slope = (np.nan_to_num(last_o) - oc) / np.maximum(x - np.nan_to_num(last_x), 1e-3)   # + = moving right
                track = np.degrees(np.arctan(-slope))                          # + = moving left (like psi)
                body = 0.5 * (np.nan_to_num(last_a) + ac)                       # camera angle ~ body heading vs strip
                c_meas = np.clip(track - body, -15, 15)
                dc = pol["crab_k"] * (c_meas - crab_est)
                crab_est = np.where(have, crab_est + dc, crab_est)
                straight = np.where(have, straight - dc, straight)
            last_x = np.where(plaus, x, last_x); last_o = np.where(plaus, oc, last_o); last_a = np.where(plaus, ac, last_a)
            tgt = np.where(plaus & (merge_left <= 0), straight, tgt)
            next_chk = np.where(chk, next_chk + pol["check_every"], next_chk)
        running = running & (x < S["L"] - 0.5)                             # LiDAR: stop 0.5 m before the wall
    yend = np.abs(y)
    return dict(yend=yend, ymax=ymax, ok=yend <= S["tol"], snaps=snaps)


def score(pol, rng, n=400, weights=(("indoor_2m", 1.0), ("plaza1_36m", 1.0), ("grooved_24m", 0.5)), stress=None):
    J = 0.0; out = {}
    for sc, w in weights:
        r = episodes(pol, sc, n, rng, stress)
        tol = SCEN[sc]["tol"]
        fail = 1 - r["ok"].mean()
        J += w * (fail + 0.3 * np.mean(r["yend"]) / tol + 0.002 * r["snaps"].mean())
        out[sc] = dict(success=r["ok"].mean(), yend_med=np.median(r["yend"]), yend_p90=np.percentile(r["yend"], 90),
                       snaps=r["snaps"].mean())
    return J, out


SPACE = dict(n_align=(1, 5, int), verify=(0, 1, int), D_merge=(0.3, 3.0, float), check_every=(0.5, 40.0, "log"),
             n_check=(1, 3, int), gA=(0.0, 1.0, float), gO=(0.0, 1.0, float), max_corr=(1.0, 15.0, float),
             ang_tol=(5.0, 30.0, float), off_tol=(0.05, 1.0, float), use_prior=(0, 1, int), crab_k=(0.0, 1.0, float))


def decode(z):
    p = {}
    for i, (k, (lo, hi, kind)) in enumerate(SPACE.items()):
        u = 1 / (1 + np.exp(-z[i]))                                        # 0..1
        if kind == "log": p[k] = float(np.exp(np.log(lo) + u * (np.log(hi) - np.log(lo))))
        elif kind is int: p[k] = int(round(lo + u * (hi - lo)))
        else: p[k] = float(lo + u * (hi - lo))
    return p


def cem(iters=12, pop=30, elite=6, seed=0):
    """Cross-entropy method = derivative-free policy search (a simple form of reinforcement learning):
    sample policies, run them in the randomised simulator, keep the best, move the sampling distribution."""
    rng = np.random.default_rng(seed)
    d = len(SPACE); mu = np.zeros(d); sd = np.full(d, 1.5); best = (1e9, None)
    hist = []
    for it in range(iters):
        Z = mu + sd * rng.standard_normal((pop, d))
        scores = np.array([score(decode(z), np.random.default_rng(1000 + it), n=150)[0] for z in Z])
        idx = np.argsort(scores)[:elite]
        mu = Z[idx].mean(0); sd = Z[idx].std(0) + 0.05
        if scores[idx[0]] < best[0]: best = (scores[idx[0]], Z[idx[0]])
        hist.append(scores[idx[0]])
        print(f"  CEM iteration {it + 1:2d}: best cost {scores[idx[0]]:.3f}")
    return decode(best[1]), hist


if __name__ == "__main__":
    import json, sys
    rng = np.random.default_rng(7)
    BASE = dict(n_align=1, verify=0, D_merge=1.0, check_every=1000.0, n_check=1, gA=0.5, gO=0.5, max_corr=10.0, ang_tol=30.0, off_tol=1.0, use_prior=0, crab_k=0.0)
    cand = {
        "A gyro only, no camera at all": dict(BASE, n_align=1, check_every=1000.0) | {"_dead": True},
        "B align once (1 snap) + straight": BASE,
        "C align (3 snaps, median) + verify + straight": dict(BASE, n_align=3, verify=1),
        "D C + check every 2 m": dict(BASE, n_align=3, verify=1, check_every=2.0, n_check=1),
        "E C + check every 5 m, 3 snaps/check": dict(BASE, n_align=3, verify=1, check_every=5.0, n_check=3, ang_tol=15, off_tol=0.5),
        "F C + measured crab prior, no checks": dict(BASE, n_align=3, verify=1, use_prior=1),
        "G F + check every 2 m + crab learning": dict(BASE, n_align=3, verify=1, use_prior=1, check_every=2.0, crab_k=0.5),
    }
    res = {}
    print("hand-made strategies (400 random episodes per scenario):")
    for name, pol in cand.items():
        stress = "camera_dead" if pol.pop("_dead", False) else None
        J, out = score(pol, np.random.default_rng(1), n=400, stress=stress)
        res[name] = dict(cost=J, **{k: {kk: round(float(vv), 3) for kk, vv in v.items()} for k, v in out.items()})
        print(f"\n{name}: cost {J:.3f}")
        for sc, o in out.items():
            print(f"   {sc:12s} on-strip at the end {o['success'] * 100:5.1f} %   |y_end| median {o['yend_med'] * 100:5.1f} cm  p90 {o['yend_p90'] * 100:5.1f} cm   snaps {o['snaps']:.1f}")
    print("\npolicy search (cross-entropy method):")
    best, hist = cem()
    J, out = score(best, np.random.default_rng(1), n=1000)
    print("\nBEST policy:", json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in best.items()}))
    for sc, o in out.items():
        print(f"   {sc:12s} on-strip at the end {o['success'] * 100:5.1f} %   |y_end| median {o['yend_med'] * 100:5.1f} cm  p90 {o['yend_p90'] * 100:5.1f} cm   snaps {o['snaps']:.1f}")
    print("\nfailure / stress tests of the BEST policy (1000 episodes each):")
    stress_res = {}
    for st in [None, "camera_dead", "link_50pct", "bad_scale", "mount_8deg", "crab_x2", "sign_flip"]:
        J, out = score(best, np.random.default_rng(3), n=1000, stress=st)
        stress_res[str(st)] = {k: {kk: round(float(vv), 3) for kk, vv in v.items()} for k, v in out.items()}
        print(f"  {str(st):12s}: " + " | ".join(f"{sc} {o['success'] * 100:4.0f} % ({o['yend_med'] * 100:4.0f} cm)" for sc, o in out.items()))
    json.dump(dict(hand=res, best=best, cem=hist, stress=stress_res), open("sim_results.json", "w"), indent=1, default=float)
