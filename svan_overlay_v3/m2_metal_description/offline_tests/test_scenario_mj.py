"""Run the REAL svan_walk_controller.py (fakeros) + MuJoCo for the two comparison scenarios and check the svan_log v1 it writes.
Usage:  python3 test_scenario_mj.py [outdir]      -> outdir/svan_stand_mujoco.csv, outdir/svan_trot_mujoco.csv  (+ PASS/FAIL lines)"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_node_mj as T
from fakeros import BUS
import svan_log as L

out = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "scenario_out")
os.makedirs(out, exist_ok=True)
ok_all = True


def check(name, cond, info=""):
    global ok_all
    ok_all &= bool(cond)
    print("  %-4s %s %s" % ("PASS" if cond else "FAIL", name, info))


for scn, T_end in (("stand", 6.0), ("trot", 12.0)):
    path = os.path.join(out, "svan_%s_mujoco.csv" % scn)
    if os.path.exists(path):
        os.remove(path)
    res, node = T.run("none", W=[(50.0, 0.0)], t_max=T_end + 0.5, quiet=True,
                      extra=dict(scenario=scn, log_csv=path, log_source="mujoco", waypoints=[50.0, 0.0]))
    pub = BUS.published["/effort_controller/commands"]
    print("scenario %s: %s" % (scn, path))
    check("file closed by the node at T", node.slog is not None and not node.slog.open and node.slog.n == int(T_end / 0.01) + 1, "(%d rows)" % node.slog.n)
    lg = L.Log(path)
    dt = np.diff(lg.t)
    check("time starts at 0, step 0.01 s", abs(lg.t[0]) < 1e-9 and np.allclose(dt, 0.01, atol=1e-6), "(t_end=%.3f)" % lg.t[-1])
    check("no NaN except Fn", np.isfinite(np.delete(lg.data, [10, 11, 12, 13], axis=1)).all())
    # latency convention: the logged tau at tick n is the command published at tick n-1
    n = np.round(lg.t / 0.002).astype(int)
    pubm = np.array([m.data for m in pub])
    check("tau column = command of the previous tick (1 tick latency)", np.allclose(lg.tau[1:], pubm[n[1:] - 1], atol=1e-4))
    check("first tau row is zero (nothing applied yet)", np.allclose(lg.tau[0], 0))
    if scn == "stand":
        check("stays at origin", np.hypot(*(lg.pos[-1, :2])) < 0.02, "(x,y)=(%.3f,%.3f)" % tuple(lg.pos[-1, :2]))
        check("cmd is zero", np.allclose(lg.cmd, 0))
        ff = np.array(T.C.FF_STAND)
        check("static torque ~ FF_STAND at 5.5 s (|err| < 0.6 Nm)", np.max(np.abs(lg.tau[-50:].mean(0) - ff)) < 0.6,
              "(max err %.2f)" % np.max(np.abs(lg.tau[-50:].mean(0) - ff)))
    else:
        i4 = np.searchsorted(lg.t, 4.0)
        check("cmd_vx is 0 before 4 s, 0.20 from 4 s", np.allclose(lg.cmd[:i4, 0], 0) and np.allclose(lg.cmd[i4:, 0], 0.20) and np.allclose(lg.cmd[:, 1:], 0))
        v = (lg.pos[-1, 0] - lg.pos[np.searchsorted(lg.t, 5.0), 0]) / (lg.t[-1] - 5.0)
        check("walks forward at about 0.20 m/s (5-12 s mean)", 0.15 < v < 0.25, "(%.3f m/s)" % v)
        check("stays upright", np.max(np.abs(lg.rpy[:, :2])) < 0.3, "(max |roll|,|pitch| = %.3f rad)" % np.max(np.abs(lg.rpy[:, :2])))
        check("no torque saturation", np.max(np.abs(lg.tau)) < 23.84 - 1e-6, "(max %.2f Nm)" % np.max(np.abs(lg.tau)))
print("ALL PASS" if ok_all else "SOME FAILED")
sys.exit(0 if ok_all else 1)
