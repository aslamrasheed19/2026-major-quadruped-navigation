"""mj_sim.py -- offline test rig: the REAL svan_core.SvanController + svan_nav.Navigator driving the Svan M2 in MuJoCo.

Not Gazebo.  Same masses / inertias / joint limits / collision primitives as the URDF, same 500 Hz torque loop with one tick
of latency, ground-truth pose (like the p3d plugin), a 2-D lidar done with mj_ray (so body tilt and ground hits are real).
"""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

import os
import shutil
import subprocess
import tempfile

PKG = os.environ.get("SVAN_PKG") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))      # the xterra_m2_description folder
sys.path.insert(0, os.path.join(PKG, "scripts"))
import compact_urdf as CU          # noqa: E402
import svan_core as C              # noqa: E402
import svan_nav as N               # noqa: E402

URDF = os.environ.get("SVAN_URDF") or os.path.join(PKG, "urdf", "m2_metal_description.urdf")
TMP = tempfile.mkdtemp(prefix="svan_mj_")
LIDAR_XYZ = np.array([0.05, 0.0, 0.158])          # lidar_link origin in base_link
LIDAR_MASS = 0.83
LIDAR_R, LIDAR_H = 0.0515, 0.072
ROBOT_MASS = 12.659


def _lidar_inertia():
    m, r, h = LIDAR_MASS, LIDAR_R, LIDAR_H
    ixx = m * (3 * r * r + h * h) / 12.0
    return np.diag([ixx, ixx, 0.5 * m * r * r])


def build_model(circles=(), boxes=(), lidar=False, height=0.6, floor_mu=1.0):
    TAG = "p%d" % os.getpid()          # per-process temp names (evaluated at call time: forked workers must not share model files)
    if shutil.which("xacro"):                           # the real file may contain xacro tags
        root = ET.fromstring(subprocess.run(["xacro", URDF], check=True, capture_output=True, text=True).stdout)
    else:
        root = ET.parse(URDF).getroot()
    CU.replace_collisions(root)
    CU.set_dynamics(root, 1.0, 0.0)
    for l in root.findall("link"):
        for v in l.findall("visual"):
            l.remove(v)
    for tag in ("transmission", "ros2_control", "gazebo"):
        for e in root.findall(tag):
            root.remove(e)
    open(TMP + "/svan_nocol_%s.urdf" % TAG, "w").write(ET.tostring(root, encoding="unicode"))
    m0 = mujoco.MjModel.from_xml_path(TMP + "/svan_nocol_%s.urdf" % TAG)
    mujoco.mj_saveLastXML(TMP + "/svan_saved_%s.xml" % TAG, m0)

    tree = ET.parse(TMP + "/svan_saved_%s.xml" % TAG); mj = tree.getroot()
    for e in mj.findall("compiler"):
        mj.remove(e)
    ET.SubElement(mj, "compiler", angle="radian")          # joint ranges in the saved XML are radians (default would be degrees)
    ET.SubElement(mj, "option", timestep="0.001", gravity="0 0 -9.81")
    wb = mj.find("worldbody")
    kids = list(wb)
    for k in kids:
        wb.remove(k)
    base = ET.SubElement(wb, "body", name="base", pos="0 0 0.258")
    ET.SubElement(base, "freejoint", name="root")
    # base inertial from the URDF (base_link), optionally merged with the lidar
    m_b = 6.4642; c_b = np.array([0.12637, 0.00021815, 0.02595])
    I_b = np.array([[0.003859, -3.6436e-06, -7.6209e-05], [-3.6436e-06, 0.0054555, -1.7659e-06], [-7.6209e-05, -1.7659e-06, 0.007064]])
    if lidar:
        mt = m_b + LIDAR_MASS; c = (m_b * c_b + LIDAR_MASS * LIDAR_XYZ) / mt
        def pa(mm, d):
            return mm * ((d @ d) * np.eye(3) - np.outer(d, d))
        I = I_b + pa(m_b, c_b - c) + _lidar_inertia() + pa(LIDAR_MASS, LIDAR_XYZ - c)
        m_b, c_b, I_b = mt, c, I
    ET.SubElement(base, "inertial", pos=" ".join("%.6f" % v for v in c_b), mass="%.5f" % m_b,
                  fullinertia=" ".join("%.7g" % v for v in (I_b[0, 0], I_b[1, 1], I_b[2, 2], I_b[0, 1], I_b[0, 2], I_b[1, 2])))
    for k in kids:
        base.append(k)
    for g in base.iter("geom"):
        g.set("group", "1")                          # robot geoms: group 1 (the lidar rays only look at group 0)
    if lidar:
        ET.SubElement(base, "geom", type="cylinder", size="%g %g" % (LIDAR_R, LIDAR_H / 2), pos=" ".join(map(str, LIDAR_XYZ)),
                      contype="0", conaffinity="0", group="2", mass="0", rgba="0.1 0.1 0.1 1")
    # world
    ET.SubElement(wb, "geom", name="floor", type="plane", size="20 20 0.1", friction="%g 0.005 0.0001" % floor_mu, group="0")
    for i, (cx, cy, r) in enumerate(circles):
        ET.SubElement(wb, "geom", name="obs_cyl_%d" % i, type="cylinder", size="%g %g" % (r, height / 2), pos="%g %g %g" % (cx, cy, height / 2), group="0")
    for i, (cx, cy, w, h) in enumerate(boxes):
        ET.SubElement(wb, "geom", name="obs_box_%d" % i, type="box", size="%g %g %g" % (w / 2, h / 2, height / 2), pos="%g %g %g" % (cx, cy, height / 2), group="0")
    act = ET.SubElement(mj, "actuator")
    for j, n in enumerate(CU.JOINT_ORDER):
        lim = C.TAU_LIM[j]
        ET.SubElement(act, "motor", name=n, joint=n, ctrlrange="%g %g" % (-lim, lim), gear="1")
    tree.write(TMP + "/svan_world_%s.xml" % TAG)
    return mujoco.MjModel.from_xml_path(TMP + "/svan_world_%s.xml" % TAG)


class Rig:
    def __init__(self, circles=(), boxes=(), lidar=False, ff_scale=None, seed=0, spawn=(0.0, 0.0, 0.0), scan_noise=0.01, scan_period=0.1, **ctrl_kw):
        self.m = build_model(circles, boxes, lidar); self.d = mujoco.MjData(self.m)
        self.circles, self.boxes, self.lidar = list(circles), list(boxes), lidar
        self.rng = np.random.default_rng(seed); self.scan_noise = scan_noise; self.scan_period = scan_period
        self.jq = [self.m.joint(n).qposadr[0] for n in CU.JOINT_ORDER]; self.jv = [self.m.joint(n).dofadr[0] for n in CU.JOINT_ORDER]
        self.base_id = self.m.body("base").id
        q_stand = np.array(C.stand_joints(), float).reshape(-1)
        self.d.qpos[0:3] = [spawn[0], spawn[1], 0.258]
        yaw = spawn[2]; self.d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        for a, q in zip(self.jq, q_stand):
            self.d.qpos[a] = q
        mujoco.mj_forward(self.m, self.d)
        mass_scale = (ROBOT_MASS + (LIDAR_MASS if lidar else 0.0)) / ROBOT_MASS
        self.ctl = C.SvanController(ff_scale=(ff_scale if ff_scale is not None else mass_scale), **ctrl_kw)
        self.ctl.start(0.0, self.q())
        self.tau_prev = np.zeros(12); self.k = 0
        # clearance bookkeeping
        self.robot_geoms = [g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] != 0 and self.m.geom_contype[g] != 0]
        self.obs_geoms = [g for g in range(self.m.ngeom) if (self.m.geom(g).name or "").startswith("obs_")]
        self.min_clear = 9.9
        self.scan = None; self.t_scan = -1e9
        self._ray_dirs_local = None

    # ---- state ---------------------------------------------------------------------------------
    def t(self):
        return self.d.time

    def q(self):
        return np.array([self.d.qpos[a] for a in self.jq])

    def dq(self):
        return np.array([self.d.qvel[a] for a in self.jv])

    def pose(self):
        p = self.d.qpos[0:3]; w, x, y, z = self.d.qpos[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        R = self.d.xmat[self.base_id].reshape(3, 3)
        tilt = math.acos(max(-1.0, min(1.0, R[2, 2])))
        return float(p[0]), float(p[1]), float(yaw), float(tilt)

    # ---- lidar ---------------------------------------------------------------------------------
    def lidar_scan(self, n=360, rmin=0.15, rmax=6.0):
        R = self.d.xmat[self.base_id].reshape(3, 3); p = self.d.xpos[self.base_id]
        origin = p + R @ LIDAR_XYZ
        amin = -math.pi; inc = 2 * math.pi / (n - 1)
        if self._ray_dirs_local is None:
            a = amin + inc * np.arange(n); self._ray_dirs_local = np.stack([np.cos(a), np.sin(a), np.zeros(n)], 1)
        dirs = (R @ self._ray_dirs_local.T).T
        out = np.empty(n); gid = np.zeros(1, np.int32); grp = np.array([1, 0, 0, 0, 0, 0], np.uint8)
        for i in range(n):
            dist = mujoco.mj_ray(self.m, self.d, origin, dirs[i], grp, 1, self.base_id, gid)
            if dist < 0 or dist > rmax:
                out[i] = np.inf
            elif dist < rmin:
                out[i] = -np.inf
            else:
                out[i] = dist + self.rng.normal(0.0, self.scan_noise)
        return out, amin, inc

    # ---- clearance -----------------------------------------------------------------------------
    def clearance(self):
        best = 9.9; ft = np.zeros(6)
        for go in self.obs_geoms:
            for gr in self.robot_geoms:
                dd = mujoco.mj_geomDistance(self.m, self.d, gr, go, 0.6, ft)
                best = min(best, dd)
        return best

    # ---- stepping ------------------------------------------------------------------------------
    def tick(self, vx=0.0, wz=0.0):
        """One 2 ms control tick (2 physics steps).  The torque computed now is applied on the NEXT tick (1 tick latency)."""
        self.ctl.command(vx, 0.0, wz)
        tau, _ = self.ctl.step(self.d.time, self.q(), self.dq())
        for _ in range(2):
            self.d.ctrl[:] = self.tau_prev
            mujoco.mj_step(self.m, self.d)
        self.tau_prev = tau
        self.k += 1
        if self.obs_geoms and self.k % 10 == 0:
            self.min_clear = min(self.min_clear, self.clearance())
        return tau


def run_mission(mode, waypoints, circles, boxes, t_start=6.0, t_max=400.0, lidar=None, seed=0, nav_hz=20.0, verbose=True, nav_kw=None, rig_kw=None, trace_dt=0.1):
    """mode: 'known' | 'sensed' | 'plain' (WaypointFollower, ignores obstacles).  Returns a result dict."""
    lidar = (mode == "sensed") if lidar is None else lidar
    rig = Rig(circles, boxes, lidar=lidar, seed=seed, **(rig_kw or {}))
    nav_kw = nav_kw or {}
    if mode == "plain":
        nav = None; wf = C.WaypointFollower(waypoints)
    else:
        nav = N.Navigator(mode, waypoints, circles=circles if mode == "known" else (), boxes=boxes if mode == "known" else (), **nav_kw)
    flip = C.FlipMonitor()
    trace = []; tilt_max = 0.0; tau_max = 0.0; vx = wz = 0.0; status = "run"; next_nav = t_start; next_scan = t_start - 1.0; scan = None
    next_trace = 0.0; flipped = False; finish = None; gt_clear_events = 0
    nticks = int(t_max / 0.002)
    for _ in range(nticks):
        t = rig.t()
        if t >= t_start and t >= next_nav:
            x, y, yaw, tilt = rig.pose()
            next_nav += 1.0 / nav_hz
            if nav is None:
                vx, wz = wf.update(x, y, yaw); status = "done" if wf.done else "run"
            else:
                if lidar and t >= next_scan:
                    scan = rig.lidar_scan(); next_scan += rig.scan_period
                    this_scan = scan
                else:
                    this_scan = None
                vx, wz, status = nav.update(t, x, y, yaw, tilt, this_scan)
            if status == "done" and finish is None:
                finish = t
            if status == "no_path":
                vx = wz = 0.0
        tau = rig.tick(vx, wz)
        tau_max = max(tau_max, float(np.max(np.abs(tau))))
        x, y, yaw, tilt = rig.pose(); tilt_max = max(tilt_max, tilt)
        if flip.update(t, tilt):
            flipped = True; break
        if t >= next_trace:
            trace.append((t, x, y, yaw, tilt, rig.min_clear)); next_trace += trace_dt
        if finish is not None and t > finish + 1.0:
            break
        if status == "no_path" and t > t_start + 5.0 and nav is not None and nav.status == "no_path" and t - nav.t_plan > 5.0:
            break
    x, y, yaw, tilt = rig.pose()
    res = dict(mode=mode, finished=finish is not None, t_finish=finish, end=(x, y), yaw=yaw, flipped=flipped, tilt_max=tilt_max, tau_max=tau_max,
               min_clear=rig.min_clear, t_end=rig.t(), replans=(nav.replans if nav else 0), log=(nav.log if nav else []), trace=np.array(trace),
               grid=(nav.grid.occ.copy() if nav is not None else None), grid_meta=((nav.grid.x0, nav.grid.y0, nav.grid.res) if nav is not None else None),
               path=(nav.path if nav is not None else None), status=status)
    if verbose:
        print("%-7s finished=%s t=%.1f end=(%.2f,%.2f) flipped=%s tilt_max=%.1fdeg |tau|max=%.1f min_clear=%.3f m replans=%d" % (
            mode, res["finished"], res["t_end"], x, y, flipped, math.degrees(tilt_max), tau_max, rig.min_clear, res["replans"]))
    return res


if __name__ == "__main__":
    import time
    t0 = time.time()
    rig = Rig()
    print("model ok: nq", rig.m.nq, "mass", rig.m.body_mass.sum(), "obs", len(rig.obs_geoms), "robot geoms", len(rig.robot_geoms))
    for i in range(int(8.0 / 0.002)):
        rig.tick(0.0, 0.0)
    print("after 8 s standing: pose", rig.pose(), "t=", rig.t(), "wall", round(time.time() - t0, 1), "s")
