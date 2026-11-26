"""svan_log.py -- the common log format ("svan_log v1") for comparing Gazebo, MATLAB and Simscape runs of the Svan M2.

ROS-free (numpy only).  Used by svan_walk_controller.py to write a log, and by tools/svan_compare.py to read / check / compare.

One row per 10 ms of simulation time; text CSV with '#' comment lines first, then one header line, then the numbers:

    t                          s, time since the controller started (stand-up ramp starts at t = 0)
    base_x base_y base_z       m, base_link position in the world frame (ground truth)
    roll pitch yaw             rad, base_link orientation (Z-Y-X, yaw about world z)
    cmd_vx cmd_vy cmd_wz       m/s, m/s, rad/s -- the REQUESTED body velocity (before the controller's rate limiter)
    Fn_FR Fn_FL Fn_RR Fn_RL    N, foot normal force (NaN where the simulator does not provide it, e.g. Gazebo without contact sensors)
    q_<leg>_<joint> (12)       rad, measured joint angle at time t
    dq_<leg>_<joint> (12)      rad/s, measured joint speed at time t
    qref_<leg>_<joint> (12)    rad, reference angle the controller computed at time t (output of the inverse kinematics)
    tau_<leg>_<joint> (12)     N.m, torque APPLIED during the 2 ms tick that starts at t
                               (= the torque the controller computed one tick earlier: one tick of latency in all simulators)

Leg order FR, FL, RR, RL; joint order hip, thigh, calf, i.e. FR_hip, FR_thigh, FR_calf, FL_hip, ...
"""
import math
import os

import numpy as np

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
JOINT_NAMES = ["%s_%s" % (l, j) for l in LEGS for j in JOINTS]

HEADER = (["t", "base_x", "base_y", "base_z", "roll", "pitch", "yaw", "cmd_vx", "cmd_vy", "cmd_wz"]
          + ["Fn_%s" % l for l in LEGS]
          + ["q_" + n for n in JOINT_NAMES] + ["dq_" + n for n in JOINT_NAMES]
          + ["qref_" + n for n in JOINT_NAMES] + ["tau_" + n for n in JOINT_NAMES])
NCOL = len(HEADER)                                   # 62

LOG_DT = 0.01                                        # s
# The two comparison scenarios (see COMPARISON_PROTOCOL.md).  Both start with the controller's own 3 s stand-up ramp.
SCENARIOS = {
    "stand": dict(T=6.0, vx=0.0, t_walk=None),       # stand pose held; checks the gravity feed-forward / static torques
    "trot": dict(T=12.0, vx=0.20, t_walk=4.0),       # open-loop straight trot at 0.20 m/s from t = 4 s
}


def quat_to_rpy(w, x, y, z):
    """roll, pitch, yaw [rad] of a unit quaternion (w, x, y, z), Z-Y-X convention (same as the MATLAB svan_simulate)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class CsvLog:
    """Writes svan_log v1 rows at LOG_DT.  call write(...) at every control tick; it keeps only every 10 ms."""

    def __init__(self, path, source, scenario, meta=None, dt=LOG_DT):
        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path, self.dt, self.n, self.next_t = path, float(dt), 0, 0.0
        self.f = open(path, "w")
        self.f.write("# svan_log v1\n# source=%s scenario=%s log_dt=%g\n" % (source, scenario, self.dt))
        for k, v in (meta or {}).items():
            self.f.write("# %s=%s\n" % (k, v))
        self.f.write(",".join(HEADER) + "\n")

    @property
    def open(self):
        return self.f is not None

    def write(self, t, pos, quat_wxyz, cmd, q, dq, qref, tau, Fn=None):
        """Returns True when a row was written.  pos = (x, y, z); quat_wxyz = (w, x, y, z); cmd = (vx, vy, wz);
        q, dq, qref, tau = 12-vectors; Fn = 4-vector or None (written as NaN)."""
        if self.f is None or t < self.next_t - 1e-6:
            return False
        r, p, y = quat_to_rpy(*quat_wxyz)
        row = [t, pos[0], pos[1], pos[2], r, p, y, cmd[0], cmd[1], cmd[2]]
        row += list(Fn) if Fn is not None else [float("nan")] * 4
        row += list(q) + list(dq) + list(qref) + list(tau)
        assert len(row) == NCOL
        self.f.write(",".join("nan" if (isinstance(v, float) and v != v) else "%.6g" % v for v in row) + "\n")
        self.n += 1
        self.next_t += self.dt
        if self.n % 100 == 0:
            self.f.flush()
        return True

    def close(self):
        if self.f is not None:
            self.f.close(); self.f = None


class Log:
    """A loaded log.  Columns are available as arrays by name (log['tau_FR_calf']) and in groups (log.q, log.tau: N x 12)."""

    def __init__(self, path):
        self.path = path
        self.meta = {}
        with open(path) as f:
            lines = f.read().splitlines()
        hdr = None
        rows = []
        for ln in lines:
            if not ln.strip():
                continue
            if ln.startswith("#"):
                for tok in ln[1:].split():
                    if "=" in tok:
                        k, v = tok.split("=", 1); self.meta[k] = v
                continue
            if hdr is None:
                hdr = ln.split(","); continue
            rows.append([float(v) for v in ln.split(",")])
        if hdr is None:
            raise ValueError("%s: no header line" % path)
        if hdr != HEADER:
            miss = [h for h in HEADER if h not in hdr]
            raise ValueError("%s: columns differ from svan_log v1 (missing: %s)" % (path, ", ".join(miss[:6]) or "none; extra/ordered differently"))
        self.data = np.array(rows, float).reshape(-1, NCOL)
        self.col = {h: i for i, h in enumerate(HEADER)}

    def __getitem__(self, name):
        return self.data[:, self.col[name]]

    def _grp(self, prefix):
        return self.data[:, [self.col["%s_%s" % (prefix, n)] for n in JOINT_NAMES]]

    t = property(lambda s: s["t"])
    pos = property(lambda s: s.data[:, 1:4])
    rpy = property(lambda s: s.data[:, 4:7])
    cmd = property(lambda s: s.data[:, 7:10])
    Fn = property(lambda s: s.data[:, 10:14])
    q = property(lambda s: s._grp("q"))
    dq = property(lambda s: s._grp("dq"))
    qref = property(lambda s: s._grp("qref"))
    tau = property(lambda s: s._grp("tau"))


def write_matrix(path, source, scenario, t, pos, rpy, cmd, Fn, q, dq, qref, tau, meta=None):
    """Write whole arrays (rows = samples) as a v1 log -- for converters (MATLAB .mat, other tools)."""
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    A = np.column_stack([t, pos, rpy, cmd, Fn, q, dq, qref, tau])
    assert A.shape[1] == NCOL, A.shape
    with open(path, "w") as f:
        f.write("# svan_log v1\n# source=%s scenario=%s log_dt=%g\n" % (source, scenario, LOG_DT))
        for k, v in (meta or {}).items():
            f.write("# %s=%s\n" % (k, v))
        f.write(",".join(HEADER) + "\n")
        for r in A:
            f.write(",".join("nan" if v != v else "%.6g" % v for v in r) + "\n")
