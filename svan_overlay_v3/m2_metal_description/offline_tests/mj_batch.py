"""Batch robustness test: random obstacle layouts x sensor variants, sensed + known modes."""
import math, sys, json, numpy as np
from multiprocessing import Pool
import mj_sim as S

W = [(2, 0), (2, 2), (0, 2), (0, 0)]


def layout(seed, n_obs=4):
    rng = np.random.default_rng(100 + seed)
    pts = [(0, 0)] + W
    cir, box, placed = [], [], []
    for _ in range(2000):
        if len(placed) >= n_obs:
            break
        x, y = rng.uniform(0.3, 2.6), rng.uniform(-0.4, 2.4)
        if rng.random() < 0.5:
            r = rng.uniform(0.1, 0.25); ext = r; item = ("c", (round(x, 2), round(y, 2), round(r, 2)))
        else:
            w, h = rng.uniform(0.2, 0.5), rng.uniform(0.2, 0.5); ext = 0.5 * math.hypot(w, h); item = ("b", (round(x, 2), round(y, 2), round(w, 2), round(h, 2)))
        if any(math.hypot(x - a, y - b) < ext + 0.65 for a, b in pts):
            continue                                              # keep waypoints (and the start) reachable
        if any(math.hypot(x - a, y - b) < ext + e2 + 0.85 for a, b, e2 in placed):
            continue                                              # keep gaps wide enough for the 0.76 m inflated robot
        placed.append((x, y, ext))
        (cir if item[0] == "c" else box).append(item[1])
    return cir, box


VARIANTS = {
    "base":   dict(rig_kw=dict(scan_noise=0.01, scan_period=0.1), nav_kw={}),
    "short2m": dict(rig_kw=dict(scan_noise=0.01, scan_period=0.1), nav_kw=dict(r_max=2.0)),
    "noisy5Hz": dict(rig_kw=dict(scan_noise=0.05, scan_period=0.2), nav_kw={}),
}


def job(a):
    mode, seed, var = a
    cir, box = layout(seed)
    v = VARIANTS[var]
    r = S.run_mission(mode, W, cir, box, t_max=400, seed=seed, verbose=False, nav_kw=v["nav_kw"], rig_kw=v["rig_kw"])
    return dict(mode=mode, seed=seed, var=var, n_obs=len(cir) + len(box), ok=bool(r["finished"]), t=r["t_end"], flipped=bool(r["flipped"]), min_clear=r["min_clear"],
                replans=r["replans"], tilt=math.degrees(r["tilt_max"]), status=r["status"], end=r["end"], log=[m for _, m in r["log"] if "no path" in m][:2])


if __name__ == "__main__":
    jobs = [("sensed", s, v) for s in range(8) for v in VARIANTS] + [("known", s, "base") for s in range(8)]
    with Pool(int(sys.argv[1]) if len(sys.argv) > 1 else 4) as p:
        res = p.map(job, jobs)
    json.dump(res, open("batch_results.json", "w"), indent=1)
    for r in res:
        print("%-6s seed %d %-8s obs=%d ok=%-5s t=%6.1f min_clear=%6.3f replans=%2d tilt=%4.1f flipped=%s %s" % (
            r["mode"], r["seed"], r["var"], r["n_obs"], r["ok"], r["t"], r["min_clear"], r["replans"], r["tilt"], r["flipped"], r["log"]))
    for mode, var in [("sensed", v) for v in VARIANTS] + [("known", "base")]:
        rr = [r for r in res if r["mode"] == mode and r["var"] == var]
        print("%-6s %-8s success %d/%d  min clearance over all runs %.3f m" % (mode, var, sum(r["ok"] for r in rr), len(rr), min(r["min_clear"] for r in rr)))
