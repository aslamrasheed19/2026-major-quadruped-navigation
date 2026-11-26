"""Drive the REAL svan_walk_controller.py (through fakeros) with MuJoCo physics.  Checks the ROS node's own logic:
parameters, nav timer, scan/odom pairing, stale-command safety, map/path publishing.  Usage: test_node_mj.py MODE [CASE]"""
import math, sys
import numpy as np, mujoco
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fakeros
fakeros.install()
import mj_sim as S                                   # also puts overlay2/scripts on sys.path
import svan_core as C
import compact_urdf as CU
from fakeros import BUS, JointState, Odometry, LaserScan

SQ = [(2, 0), (2, 2), (0, 2), (0, 0)]


def run(nav_mode, W=SQ, circles=(), boxes=(), t_max=150.0, scan_latency=0.05, extra=None, seed=0, quiet=False):
    BUS.__init__()
    lidar = nav_mode == "sensed"
    flat = lambda xs: [float(v) for t in xs for v in t]
    BUS.overrides = dict(nav_mode=nav_mode, waypoints=flat(W), obstacle_circles=flat(circles), obstacle_boxes=flat(boxes),
                         payload_kg=0.83 if lidar else 0.0, zero_offset=False, mode="path")
    BUS.overrides.update(extra or {})
    if "svan_walk_controller" in sys.modules:
        del sys.modules["svan_walk_controller"]
    import svan_walk_controller as W_
    node = W_.SvanWalkController()
    rig = S.Rig(circles, boxes, lidar=lidar, seed=seed)
    m, d = rig.m, rig.d
    names = CU.JOINT_ORDER
    stamp = lambda t: (int(t), int((t - int(t)) * 1e9))
    pending_scans = []
    tau_prev = np.zeros(12); next_odom = 0.0; next_scan = 0.0; next_timer = {}
    dt = 0.002
    tilt_max = 0.0
    for k in range(int(t_max / dt)):
        t = d.time
        # --- publish sensors to the node ------------------------------------------------------------------
        if t >= next_odom - 1e-9:
            o = Odometry(); o.header.stamp.sec, o.header.stamp.nanosec = stamp(t)
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, d.qpos[0:3])
            w, x, y, z = d.qpos[3:7]
            o.pose.pose.orientation.w, o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z = w, x, y, z
            for cb in BUS.subs.get("/ground_truth/odom", []): cb(o)
            next_odom += 0.01
        if lidar and t >= next_scan - 1e-9:
            r, amin, inc = rig.lidar_scan()
            sc = LaserScan(); sc.header.stamp.sec, sc.header.stamp.nanosec = stamp(t)
            sc.angle_min, sc.angle_increment = amin, inc; sc.ranges = [float(v) for v in r]
            pending_scans.append((t + scan_latency, sc)); next_scan += 0.1
        while pending_scans and pending_scans[0][0] <= t + 1e-9:
            _, sc = pending_scans.pop(0)
            for cb in BUS.subs.get("/scan", []): cb(sc)
        for tm in BUS.timers:                                  # sim-time timers
            if tm[2] is None: tm[2] = t
            if t >= tm[2] - 1e-9:
                tm[1](); tm[2] += tm[0]
        js = JointState(); js.header.stamp.sec, js.header.stamp.nanosec = stamp(t)
        js.name = list(names); js.position = [float(d.qpos[a]) for a in rig.jq]; js.velocity = [float(d.qvel[a]) for a in rig.jv]
        for cb in BUS.subs.get("/joint_states", []): cb(js)
        cmd = BUS.last.get("/effort_controller/commands")
        tau_new = np.array(cmd.data) if cmd is not None else np.zeros(12)
        for _ in range(2):
            d.ctrl[:] = tau_prev; mujoco.mj_step(m, d)
        tau_prev = tau_new
        if k % 5 == 0 and rig.obs_geoms: rig.min_clear = min(rig.min_clear, rig.clearance())
        x, y, yaw, tilt = rig.pose(); tilt_max = max(tilt_max, tilt)
        if node.nav is not None and node.nav.status == "done":
            t_done = locals().get("t_done", t)
            if t - t_done > 2.0: break
        elif node.nav is None and node.follower.done:
            t_done = locals().get("t_done", t)
            if t - t_done > 2.0: break
    x, y, yaw, tilt = rig.pose()
    done = (node.nav.status == "done") if node.nav is not None else node.follower.done
    res = dict(done=done, end=(x, y), t=d.time, min_clear=rig.min_clear, tilt=math.degrees(tilt_max), logs=BUS.logs,
               replans=(node.nav.replans if node.nav else 0), maps=len(BUS.published.get("/svan/obstacle_map", [])),
               paths=len(BUS.published.get("/svan/path", [])), tfs=len(BUS.published.get("/tf", [])))
    if not quiet:
        print("%-6s done=%s t=%.1f end=(%.2f,%.2f) min_clear=%.3f tilt_max=%.1f replans=%d  /svan/path msgs=%d map msgs=%d tf msgs=%d" % (
            nav_mode, done, res["t"], x, y, rig.min_clear, res["tilt"], res["replans"], res["paths"], res["maps"], res["tfs"]))
    return res, node


if __name__ == "__main__":
    mode = sys.argv[1]
    CI = [(2.0, 1.0, 0.15)]; BX = [(1.0, 0.0, 0.3, 0.3), (1.0, 2.0, 0.3, 0.5)]
    res, node = run(mode, circles=CI if mode != "none" else (), boxes=BX if mode != "none" else ())
    for lvl, msg in res["logs"]:
        if (lvl in ("warn", "error") or "nav:" in msg) and "pos=(" not in msg: print("  [%s] %s" % (lvl, msg))
