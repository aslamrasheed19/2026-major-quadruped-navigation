"""Designed hard cases (not random): wall, U-trap, narrow gap vs wide gap, obstacle sitting on a waypoint."""
import sys, math, json
from multiprocessing import Pool
import mj_sim as S

SQ = [(2, 0), (2, 2), (0, 2), (0, 0)]
CASES = {
    # long wall across the first leg: must go around its end
    "wall":      dict(W=SQ, cir=[], box=[(1.0, 0.45, 0.2, 2.1)]),
    # U-shaped pocket opening towards the robot, goal is just behind its closed end
    "u_trap":    dict(W=[(2.2, 0.0), (0.0, 0.0)], cir=[], box=[(1.0, 0.7, 1.0, 0.15), (1.0, -0.7, 1.0, 0.15), (1.55, 0.0, 0.15, 1.55)]),
    # gap 0.70 m between two cylinders (< 0.76 m inflated width) -> robot must go around
    "gap_narrow": dict(W=[(2.0, 0.0), (0.0, 0.0)], cir=[(1.0, 0.45, 0.2), (1.0, -0.45, 0.2)], box=[]),
    # gap 1.0 m -> the robot may pass through
    "gap_wide":  dict(W=[(2.0, 0.0), (0.0, 0.0)], cir=[(1.0, 0.7, 0.2), (1.0, -0.7, 0.2)], box=[]),
    # an obstacle sits on top of waypoint 1
    "on_goal":   dict(W=SQ, cir=[(2.0, 2.0, 0.2)], box=[]),
    # crowded: 7 obstacles
    "crowded":   dict(W=SQ, cir=[(0.9, 0.3, 0.15), (2.1, 0.9, 0.2), (1.3, 1.3, 0.15), (0.5, 1.6, 0.18)], box=[(1.5, -0.1, 0.3, 0.3), (1.0, 2.1, 0.4, 0.3), (0.0, 0.9, 0.3, 0.4)]),
}


def job(a):
    mode, name = a
    c = CASES[name]
    import os
    nk = dict(margin=float(os.environ["SVAN_MARGIN"])) if "SVAN_MARGIN" in os.environ else {}
    r = S.run_mission(mode, c["W"], c["cir"], c["box"], t_max=350, seed=3, verbose=False, nav_kw=nk)
    return dict(mode=mode, name=name, ok=bool(r["finished"]), t=r["t_end"], end=tuple(round(v, 2) for v in r["end"]), flipped=bool(r["flipped"]),
                min_clear=r["min_clear"], replans=r["replans"], status=r["status"], log=[m for _, m in r["log"]][-4:], tilt=math.degrees(r["tilt_max"]))


if __name__ == "__main__":
    jobs = [(m, n) for n in CASES for m in ("known", "sensed")]
    with Pool(2) as p:
        res = p.map(job, jobs)
    for r in res:
        print("%-6s %-10s ok=%-5s t=%6.1f end=%s clear=%6.3f replans=%2d status=%s flipped=%s" % (r["mode"], r["name"], r["ok"], r["t"], r["end"], r["min_clear"], r["replans"], r["status"], r["flipped"]))
        if not r["ok"]:
            print("      ", r["log"])
    json.dump(res, open("hard_results.json", "w"), indent=1)
