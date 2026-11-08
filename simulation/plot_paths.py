"""plot_paths.py - example paths of the rover for several strategies (uses strip_sim.episodes with path recording)."""
import json, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import strip_sim as s

def run_paths(pol, scen, n=12, seed=5, stress=None):
    """Copy of strip_sim.episodes that also returns the path (x, y) of each episode."""
    src = open(s.__file__).read()
    ns = {}
    code = src.split("def score(")[0].replace("def episodes(pol, scen, n, rng, stress=None):", "def episodes_rec(pol, scen, n, rng, stress=None):")
    code = code.replace("    ymax = np.abs(y)\n", "    ymax = np.abs(y)\n    PX = [x.copy()]; PY = [y.copy()]\n")
    code = code.replace("        ymax = np.maximum(ymax, np.abs(y))\n", "        ymax = np.maximum(ymax, np.abs(y)); PX.append(x.copy()); PY.append(y.copy())\n")
    code = code.replace("    return dict(yend=yend, ymax=ymax, ok=yend <= S[\"tol\"], snaps=snaps)", "    return dict(yend=yend, ok=yend <= S[\"tol\"], PX=np.array(PX), PY=np.array(PY))")
    exec(compile(code, "rec", "exec"), ns)
    return ns["episodes_rec"](pol, scen, n, np.random.default_rng(seed), stress)

if __name__ == "__main__":
    R = json.load(open("sim_results.json"))
    BASE = dict(n_align=1, verify=0, D_merge=1.0, check_every=1000.0, n_check=1, gA=0.5, gO=0.5, max_corr=10.0,
                ang_tol=30.0, off_tol=1.0, use_prior=0, crab_k=0.0)
    strategies = [
        ("B: align once (1 snap) + straight", BASE),
        ("F: align 3 snaps + crab prior, no checks", dict(BASE, n_align=3, verify=1, use_prior=1)),
        ("G: F + check every 2 m + crab learning", dict(BASE, n_align=3, verify=1, use_prior=1, check_every=2.0, crab_k=0.5)),
        ("BEST (policy search)", R["best"]),
    ]
    fig, axs = plt.subplots(len(strategies), 2, figsize=(15, 3.3 * len(strategies)))
    for i, (name, pol) in enumerate(strategies):
        for j, scen in enumerate(["indoor_2m", "plaza1_36m"]):
            r = run_paths(pol, scen)
            ax = axs[i, j]
            tol = s.SCEN[scen]["tol"]
            ax.axhspan(-tol, tol, color="0.85", label=f"on strip (+-{tol*100:.0f} cm)")
            ax.axhline(0, color="k", lw=2, label="strip")
            for k in range(r["PX"].shape[1]):
                ax.plot(r["PX"][:, k], r["PY"][:, k], color="g" if r["ok"][k] else "r", lw=1)
            ax.set_title(f"{name} | {scen}: {r['ok'].mean()*100:.0f} % on strip", fontsize=10)
            ax.set_xlabel("along the strip (m)"); ax.set_ylabel("sideways (m), + = left")
            lim = 0.4 if scen == "indoor_2m" else 4
            ax.set_ylim(-lim, lim)
            if i == 0 and j == 0: ax.legend(fontsize=8, loc="upper right")
    plt.tight_layout(); plt.savefig("paths.png", dpi=75)
    print("saved paths.png")
