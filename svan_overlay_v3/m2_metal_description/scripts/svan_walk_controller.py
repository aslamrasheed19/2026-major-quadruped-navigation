#!/usr/bin/env python3
"""svan_walk_controller.py -- stand, trot and follow waypoints with the xTerra Svan M2.

Replaces svan_pid_controller.py.  The control law lives in svan_core.py (ROS-free, tested in MuJoCo);
this file only does the ROS plumbing:

   /joint_states ------------------------------+
   /ground_truth/odom (sim only, p3d plugin) --+--> SvanController --> /effort_controller/commands
   /cmd_vel (optional, e.g. teleop_twist_keyboard)  |   (12 joint torques, Float64MultiArray)
   waypoints parameter (config/walk.yaml) ----------+

The torque is computed inside the /joint_states callback, i.e. once per controller tick (500 Hz in
simulation time).  Nothing uses the wall clock, so it behaves the same at any real-time factor.

Obstacle avoidance (nav_mode, see svan_nav.py) runs on its own timer / callback group so that a slow plan can
never delay a torque command:
   nav_mode: none    waypoint following exactly as before (obstacles ignored)
   nav_mode: known   obstacles come from the obstacle_circles / obstacle_boxes parameters (the same lists build the Gazebo world)
   nav_mode: sensed  obstacles are NOT given: they are mapped from the 2-D lidar on /scan
The nav timer writes the body-velocity command (vx, wz); the torque callback only reads it.

Comparison runs (scenario, see COMPARISON_PROTOCOL.md): scenario:=stand | trot replaces path / nav / cmd_vel by a fixed open-loop
command schedule and writes a "svan_log v1" CSV (svan_log.py) so that the same run can be compared with MATLAB and Simscape.
"""
import math
import os
import threading
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty

import svan_core as C
import svan_log as L
import svan_nav as N

try:                                            # only used to draw the robot / map in RViz
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
except Exception:                               # pragma: no cover
    TransformBroadcaster = None

ROBOT_MASS_KG = 12.659                          # URDF total (base 6.464 + 4 legs x 1.548)

JOINTS = [f"{l}_{j}_joint" for l in C.LEGS for j in ("hip", "thigh", "calf")]


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tilt_from_quat(q):
    """angle between the body z axis and the world z axis [rad]: 0 = upright, pi/2 = on its side, pi = upside down"""
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))))


class SvanWalkController(Node):
    def __init__(self):
        super().__init__("svan_walk_controller")
        P = self.declare_parameter
        P("mode", "path")                       # path | stand | cmd_vel   (cmd_vel is always honoured when messages arrive)
        P("waypoints", [1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0])   # x0,y0,x1,y1,... in the odom frame (start = origin)
        P("walk_start_time", 6.0)               # s of simulation time after start before the path begins
        P("v_max", 0.25); P("w_max", 0.8); P("k_head", 1.5); P("reach", 0.08)
        P("kp", [60.0, 80.0, 90.0]); P("kd", [0.3, 0.36, 0.45]); P("ki", 0.0)
        P("ramp_time", 3.0); P("gait_period", 0.5); P("step_height", 0.05); P("ff_scale", 1.0)
        P("zero_offset", False)                 # True when the URDF was generated with --zero-offset (joint 0 = stand pose)
        P("flip_limit_deg", 57.0)               # body tilt above this for flip_hold s = "robot is flipped"
        P("flip_hold", 1.0)
        P("auto_reset", True)                   # flipped -> go limp and call /reset_world (Gazebo), then stand up again
        P("max_resets", 3)
        P("odom_topic", "/ground_truth/odom"); P("command_topic", "/effort_controller/commands")
        # --- obstacle avoidance --------------------------------------------------------------------------------
        dyn = ParameterDescriptor(dynamic_typing=True)          # lets an empty default list be overridden by a YAML list of doubles
        P("nav_mode", "none")                                   # none | known | sensed
        try:                                                    # x, y, r,  x, y, r, ...   [m]   /   x, y, w, h,  ...  [m] (w along x, h along y)
            P("obstacle_circles", [], dyn); P("obstacle_boxes", [], dyn)
        except Exception:                                       # a rclpy that rejects an empty default list: use an all-zero "no obstacle"
            self.get_logger().warn("empty list parameters are not accepted here; using all-zero placeholders (= no obstacle)")
            P("obstacle_circles", [0.0, 0.0, 0.0]); P("obstacle_boxes", [0.0, 0.0, 0.0, 0.0])
        P("nav_bounds", [-1.5, -1.5, 3.5, 3.5])                 # xmin, ymin, xmax, ymax of the planning map [m]; must contain path + obstacles
        P("grid_res", 0.05); P("robot_radius", 0.32); P("nav_margin", 0.10); P("lookahead", 0.30); P("replan_every", 4.0)
        P("nav_rate", 20.0)                                     # Hz of simulation time
        P("scan_topic", "/scan"); P("lidar_range_max", 4.0); P("lidar_xy", [0.05, 0.0]); P("lidar_height", 0.40)
        P("payload_kg", 0.0)                                    # mass added to the robot (lidar 0.83 kg) -> scales the gravity feed-forward
        P("publish_map", True); P("publish_tf", True)
        # --- comparison run: fixed scenario + CSV log (svan_log v1) --------------------------------------------------
        P("scenario", "none")                                   # none | stand | trot   (stand: 6 s;  trot: 12 s, 0.20 m/s from t = 4 s)
        P("scenario_vx", L.SCENARIOS["trot"]["vx"]); P("scenario_t_walk", L.SCENARIOS["trot"]["t_walk"])
        P("scenario_T", 0.0)                                    # 0 = the default length of the scenario
        P("log_csv", "")                                        # "" = ~/svan_logs/svan_<scenario>_gazebo.csv
        P("log_source", "gazebo")                               # label written into the log header (the offline test uses "mujoco")
        g = lambda n: self.get_parameter(n).value

        self.mode = g("mode")
        ff_scale = float(g("ff_scale")) * (ROBOT_MASS_KG + float(g("payload_kg"))) / ROBOT_MASS_KG
        self.ctl = C.SvanController(kp=g("kp"), kd=g("kd"), ki=g("ki"), ramp_time=g("ramp_time"),
                                    period=g("gait_period"), step_height=g("step_height"), ff_scale=ff_scale)
        wp = list(g("waypoints"))
        self.follower = C.WaypointFollower(list(zip(wp[0::2], wp[1::2])), v_max=g("v_max"), w_max=g("w_max"),
                                           k_head=g("k_head"), reach=g("reach"))
        self.walk_start = g("walk_start_time")
        self.scn = str(g("scenario")); self.slog = None; self.scn_note = False
        if self.scn not in ("none", "stand", "trot"):
            raise ValueError("scenario must be none, stand or trot (got %r)" % self.scn)
        self.prev_tau = np.zeros(12)                            # torque applied during the current tick = computed one tick earlier
        self.odom_q = (1.0, 0.0, 0.0, 0.0); self.odom_pos = (0.0, 0.0, 0.0)
        if self.scn != "none":
            sc = L.SCENARIOS[self.scn]
            self.scn_T = float(g("scenario_T")) or sc["T"]
            self.scn_vx = float(g("scenario_vx")) if self.scn == "trot" else 0.0
            self.scn_t_walk = float(g("scenario_t_walk")) if self.scn == "trot" else 1e9
            path = g("log_csv") or os.path.join("~", "svan_logs", "svan_%s_gazebo.csv" % self.scn)
            self.slog = L.CsvLog(path, str(g("log_source")), self.scn, meta=dict(
                T=self.scn_T, vx=self.scn_vx, t_walk=self.scn_t_walk if self.scn == "trot" else "none", ctrl_hz=500,
                kp=",".join("%g" % v for v in g("kp")), kd=",".join("%g" % v for v in g("kd")), ramp_time=g("ramp_time"),
                gait_period=g("gait_period"), step_height=g("step_height"), ff_scale=ff_scale))
            self.get_logger().info("scenario=%s: T=%.1f s, vx=%.2f m/s from t=%.1f s, logging to %s" % (
                self.scn, self.scn_T, self.scn_vx, self.scn_t_walk if self.scn == "trot" else float("nan"), self.slog.path))
        self.q_off = self.ctl.q_stand.copy() if g("zero_offset") else np.zeros(12)

        self.idx = None                         # joint-name -> index map, built from the first message
        self.t_start = None; self.t_last = None; self.t_sim = 0.0
        self.odom = None                        # (x, y, yaw) from ground truth
        self.body = (0.0, 0.0)                  # (z, tilt) from ground truth
        self.flip = C.FlipMonitor(limit=math.radians(g("flip_limit_deg")), hold=g("flip_hold"))
        self.auto_reset = bool(g("auto_reset")); self.max_resets = int(g("max_resets"))
        self.resets_done = 0; self.t_reset = -1e9; self.t_note = -1e9
        self.state = "run"                      # run | flipped
        self.reset_cli = self.create_client(Empty, "/reset_world")
        self.dr = [0.0, 0.0, 0.0]               # dead-reckoned fallback pose
        self.warned_dr = False
        self.cmd_vel = None; self.cmd_vel_t = -1e9
        self.last_log = -1e9

        # ---- obstacle avoidance -----------------------------------------------------------------------------------
        self.nav_mode = str(g("nav_mode")); self.nav = None
        self.nav_cmd = (0.0, 0.0); self.nav_cmd_t = -1e9; self.nav_status = "-"
        self.odom_hist = deque(maxlen=300)                      # (stamp, x, y, yaw, tilt) -- to pair a scan with the pose at ITS stamp
        self.scan_msg = None; self.scan_done = None; self.scan_lock = threading.Lock()
        self.nav_log_i = 0; self.t_nav_note = -1e9; self.t_map_pub = -1e9; self.path_seen = None
        if self.nav_mode not in ("none", "known", "sensed"):
            raise ValueError("nav_mode must be none, known or sensed (got %r)" % self.nav_mode)
        self.cb_ctrl = MutuallyExclusiveCallbackGroup()         # torque loop alone in its group
        self.cb_nav = MutuallyExclusiveCallbackGroup()          # odometry, scans, planning
        self.nav_kw = dict(bounds=tuple(g("nav_bounds")), res=float(g("grid_res")), robot_radius=float(g("robot_radius")),
                           margin=float(g("nav_margin")), v_max=g("v_max"), w_max=g("w_max"), k_head=g("k_head"), reach=g("reach"),
                           lookahead=float(g("lookahead")), replan_every=float(g("replan_every")), sensor_xy=tuple(g("lidar_xy")), r_max=float(g("lidar_range_max")))
        if self.nav_mode != "none":
            self._make_nav()

        self.pub = self.create_publisher(Float64MultiArray, g("command_topic"), 10)
        # best-effort + depth 1: if we are ever late we skip stale states instead of queueing them
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, qos, callback_group=self.cb_ctrl)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, 10, callback_group=self.cb_nav)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10, callback_group=self.cb_nav)
        if self.nav_mode != "none":
            latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
            self.path_pub = self.create_publisher(Path, "/svan/path", latched)
            self.map_pub = self.create_publisher(OccupancyGrid, "/svan/obstacle_map", latched)
            if self.nav_mode == "sensed":
                self.create_subscription(LaserScan, g("scan_topic"), self.on_scan, qos_profile_sensor_data, callback_group=self.cb_nav)
            self.create_timer(1.0 / float(g("nav_rate")), self.on_nav_timer, callback_group=self.cb_nav)
            self._publish_map(force=True)                       # latched: RViz started later still gets the (known) map
        self.tf_br = TransformBroadcaster(self) if (TransformBroadcaster is not None and g("publish_tf")) else None
        self.get_logger().info(
            f"svan_walk_controller up: mode={self.mode}, nav_mode={self.nav_mode}, {len(self.follower.wps)} waypoints, "
            f"feed-forward x{ff_scale:.3f}, waiting for /joint_states ...")

    # ---- callbacks ---------------------------------------------------------------------
    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.body = (p.position.z, tilt_from_quat(p.orientation))
        self.odom_pos = (p.position.x, p.position.y, p.position.z)
        self.odom_q = (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
        self.odom_hist.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, *self.odom, self.body[1]))
        if self.tf_br is not None:                                  # world -> base, so RViz can draw the map, path and scan
            tf = TransformStamped()
            tf.header = m.header; tf.header.frame_id = "world"; tf.child_frame_id = "base"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.position.x, p.position.y, p.position.z
            tf.transform.rotation = p.orientation
            self.tf_br.sendTransform(tf)

    def on_cmd_vel(self, m):
        self.cmd_vel = (m.linear.x, m.linear.y, m.angular.z)
        self.cmd_vel_t = self.t_sim

    # ---- obstacle avoidance ----------------------------------------------------------------------------
    def _make_nav(self):
        g = lambda n: list(self.get_parameter(n).value or [])
        wp = g("waypoints"); circ = g("obstacle_circles"); box = g("obstacle_boxes")
        if len(circ) % 3 or len(box) % 4:
            raise ValueError("obstacle_circles needs triples (x, y, r), obstacle_boxes needs quadruples (x, y, w, h)")
        circles = [c for c in zip(circ[0::3], circ[1::3], circ[2::3]) if c[2] > 0.0]                     # radius 0 = placeholder
        boxes = [b for b in zip(box[0::4], box[1::4], box[2::4], box[3::4]) if b[2] > 0.0 and b[3] > 0.0]
        self.nav = N.Navigator(self.nav_mode, list(zip(wp[0::2], wp[1::2])), circles=circles, boxes=boxes, **self.nav_kw)
        self.nav_log_i = 0; self.path_seen = None; self.nav_status = "run"
        with self.scan_lock:
            self.scan_msg = None; self.scan_done = None
        if self.nav_mode == "known":
            self.get_logger().info("nav_mode=known: %d circles + %d boxes given; planning around them" % (len(circles), len(boxes)))
        else:
            self.get_logger().info("nav_mode=sensed: no obstacle is given; the map is built from %s" % self.get_parameter("scan_topic").value)
        self._publish_map(force=True)

    def on_scan(self, m):
        with self.scan_lock:
            self.scan_msg = m

    def _pose_at(self, stamp):
        """(x, y, yaw, tilt) from the odometry sample closest to `stamp`"""
        if not self.odom_hist:
            return None
        best = min(self.odom_hist, key=lambda h: abs(h[0] - stamp))
        if abs(best[0] - stamp) > 0.2:
            best = self.odom_hist[-1]
        return best[1], best[2], best[3], best[4]

    def on_nav_timer(self):
        if self.nav is None or self.odom is None or self.t_start is None or self.state != "run":
            return
        t = self.t_sim
        if t - self.t_start <= self.walk_start:                 # still standing up / waiting
            return
        if self.cmd_vel is not None and (t - self.cmd_vel_t) < 0.5:
            return                                              # /cmd_vel (teleop) has priority
        x, y, yaw = self.odom; tilt = self.body[1]
        scan = scan_pose = None
        with self.scan_lock:
            m = self.scan_msg
            if m is not None and m is not self.scan_done:
                self.scan_done = m
            else:
                m = None
        if m is not None:
            stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            scan = (np.asarray(m.ranges, float), m.angle_min, m.angle_increment)
            scan_pose = self._pose_at(stamp)
        vx, wz, st = self.nav.update(t, x, y, yaw, tilt, scan, scan_pose)
        self.nav_cmd = (vx, wz); self.nav_cmd_t = t; self.nav_status = st
        while self.nav_log_i < len(self.nav.log):
            tt, msg = self.nav.log[self.nav_log_i]; self.nav_log_i += 1
            (self.get_logger().warn if "no path" in msg else self.get_logger().info)("nav: " + msg)
        if st == "no_path" and t - self.t_nav_note > 5.0:
            self.t_nav_note = t
            self.get_logger().warn("nav: no collision-free path to waypoint %d with the obstacles known so far; standing still and retrying" % self.nav.gi)
        if self.nav.path is not self.path_seen:
            self.path_seen = self.nav.path; self._publish_path()
        if self.nav_mode == "sensed" and t - self.t_map_pub > 0.5:
            self._publish_map()

    def _publish_path(self):
        p = Path(); p.header.frame_id = "world"; p.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in (self.nav.path or []):
            ps = PoseStamped(); ps.header = p.header; ps.pose.position.x = float(x); ps.pose.position.y = float(y); ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

    def _publish_map(self, force=False):
        if self.nav is None or not hasattr(self, "map_pub"):
            return
        self.t_map_pub = self.t_sim
        gr = self.nav.grid; m = OccupancyGrid()
        m.header.frame_id = "world"; m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(gr.res); m.info.width = int(gr.nx); m.info.height = int(gr.ny)
        m.info.origin.position.x = float(gr.x0); m.info.origin.position.y = float(gr.y0); m.info.origin.orientation.w = 1.0
        known = (gr.logodds != 0.0) | gr.occ                    # unseen cells are "unknown" (-1) in sensed mode
        data = np.where(gr.occ, 100, np.where(known, 0, -1)).astype(np.int8)
        if self.nav_mode == "known":
            data = np.where(gr.occ, 100, 0).astype(np.int8)
        m.data = data.reshape(-1).tolist()
        self.map_pub.publish(m)

    def on_joint_state(self, msg):
        if self.idx is None:
            try:
                self.idx = [list(msg.name).index(j) for j in JOINTS]
            except ValueError:
                self.get_logger().error(f"/joint_states is missing some of {JOINTS}", throttle_duration_sec=2.0)
                return
        q = np.array([msg.position[i] for i in self.idx]) + self.q_off
        dq = np.array([msg.velocity[i] for i in self.idx]) if len(msg.velocity) >= len(self.idx) else np.zeros(12)

        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.t_last is None:
            t, dt = 0.0, 0.002
        else:
            ds = stamp - self.t_last
            dt = ds if 0.0 < ds < 0.05 else 0.002          # bad / zero stamp: assume the 500 Hz tick
            t = self.t_sim + dt
        self.t_last = stamp
        self.t_sim = t

        if self.t_start is None:
            self.t_start = t
            self.ctl.start(t, q)
            self.get_logger().info("first /joint_states received: ramping to the stand pose")
            err = float(np.max(np.abs(q - self.ctl.q_stand)))
            if err > 0.3:
                if self.q_off.any():        # zero-offset URDF: the robot was born standing and sagged while the controller started
                    self.get_logger().info(
                        "legs sagged %.2f rad while the effort controller was starting (normal); "
                        "standing up from the measured pose." % err)
                else:
                    self.get_logger().warn(
                        "joints are %.2f rad away from the stand pose at start: the initial pose was not applied "
                        "(or the legs sagged). Use  initial_pose:=offset  (the default in this version)." % err)

        # ---- flip supervision (needs the ground-truth odometry; silently off without it) ----------------
        if self.odom is not None:
            ev = self.flip.update(t, self.body[1])
            if ev == "flipped" and self.state == "run":
                self.state = "flipped"; self.t_note = -1e9
                self.get_logger().error(
                    "ROBOT IS FLIPPED (body tilt %.0f deg, height %.2f m) -> torques off." % (math.degrees(self.body[1]), self.body[0]))
            elif ev == "recovered" and self.state == "flipped":
                self.state = "run"
                self.t_start = t; self.ctl.start(t, q); self.follower.i = 0; self.dr = [0.0, 0.0, 0.0]
                if self.nav_mode != "none":
                    self._make_nav()                                # back at the spawn pose: forget the old map / path
                self.get_logger().info("robot is upright again -> standing up and restarting the path")
        if self.state == "flipped":
            self._flipped_step(t)
            out = Float64MultiArray(); out.data = [0.0] * 12
            self.pub.publish(out)
            return

        ts = t - self.t_start
        vx = vy = wz = 0.0
        src = "stand"
        if self.scn != "none":                                  # fixed comparison scenario: no path, no nav, no teleop
            if self.scn_t_walk - 1e-6 <= ts <= self.scn_T + 1e-6:     # same inclusive window as the MATLAB scenario (t_walk <= t <= T)
                vx = self.scn_vx
            src = "scenario:" + self.scn
        elif self.cmd_vel is not None and (t - self.cmd_vel_t) < 0.5:
            vx, vy, wz = self.cmd_vel; src = "cmd_vel"
        elif self.mode == "path" and self.nav is not None and ts > self.walk_start:
            if self.odom is not None and (t - self.nav_cmd_t) < 0.5:       # a stale command (nav thread stalled) means stop
                vx, wz = self.nav_cmd
            src = "nav:" + self.nav_status
        elif self.mode == "path" and self.nav is None and ts > self.walk_start and not self.follower.done:
            x, y, yaw = self._pose()
            vx, wz = self.follower.update(x, y, yaw); src = "path"
        self.ctl.command(vx, vy, wz)
        # open-loop pose estimate used only if no odometry is arriving
        c, s = math.cos(self.dr[2]), math.sin(self.dr[2])
        self.dr[0] += (self.ctl.vf[0] * c - self.ctl.vf[1] * s) * dt
        self.dr[1] += (self.ctl.vf[0] * s + self.ctl.vf[1] * c) * dt
        self.dr[2] += self.ctl.vf[2] * dt

        tau, q_ref = self.ctl.step(t, q, dq)
        out = Float64MultiArray(); out.data = [float(v) for v in tau]
        self.pub.publish(out)
        if self.slog is not None and self.slog.open:
            if self.odom is None and not self.scn_note:
                self.scn_note = True
                self.get_logger().warn("scenario log: no ground-truth odometry yet -> base pose columns are zero until it arrives")
            self.slog.write(ts, self.odom_pos, self.odom_q, (vx, vy, wz), q, dq, q_ref, self.prev_tau)      # q, q_ref: absolute angles (zero-offset already added)
            if ts >= self.scn_T - 1e-6:
                self.slog.close()
                self.get_logger().info("scenario %s finished: %d rows written to %s  (the robot keeps standing; Ctrl+C when done)" % (
                    self.scn, self.slog.n, self.slog.path))
        self.prev_tau = np.array(tau)

        if t - self.last_log > 2.0:
            self.last_log = t
            x, y, yaw = self._pose(quiet=True)
            self.get_logger().info(
                f"t={ts:6.1f}s  {src:12s} pos=({x:5.2f},{y:5.2f}) yaw={math.degrees(yaw):6.1f}deg "
                f"z={self.body[0]:4.2f} tilt={math.degrees(self.body[1]):4.0f}deg "
                f"wp={(self.nav.gi if self.nav is not None else min(self.follower.i, len(self.follower.wps)))}/{len(self.follower.wps)} "
                f"cmd=({vx:.2f},{wz:.2f}) |tau|max={np.abs(tau).max():.1f}Nm")

    def _flipped_step(self, t):
        """While flipped: try /reset_world (Gazebo puts the robot back at its spawn pose) up to max_resets times;
        afterwards just tell the user how to do it by hand.  Standing up again is automatic once it is upright."""
        if t - self.t_reset < 3.0:
            return                                              # give the last reset time to take effect
        if self.auto_reset and self.resets_done < self.max_resets:
            if self.reset_cli.service_is_ready():
                self.reset_cli.call_async(Empty.Request())
                self.resets_done += 1; self.t_reset = t
                self.get_logger().warn("calling /reset_world (automatic reset %d of %d)" % (self.resets_done, self.max_resets))
            elif t - self.t_note > 5.0:
                self.t_note = t
                self.get_logger().warn("/reset_world is not available (gazebo_ros_init not loaded?)")
        elif t - self.t_note > 10.0:
            self.t_note = t
            self.get_logger().warn("still flipped. Reset it by hand:  ros2 service call /reset_world std_srvs/srv/Empty   "
                                   "(or press Ctrl+R in the Gazebo window). The controller stands up by itself afterwards.")

    def _pose(self, quiet=False):
        if self.odom is not None:
            return self.odom
        if not quiet and not self.warned_dr:
            self.get_logger().warn("no odometry on %s -> dead reckoning from commanded velocity (open loop, "
                                   "will drift). Is the p3d plugin loaded?" % self.get_parameter("odom_topic").value)
            self.warned_dr = True
        return tuple(self.dr)


def main():
    rclpy.init()
    node = SvanWalkController()
    ex = MultiThreadedExecutor(num_threads=3)               # torque loop and planner never share a callback group
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.slog is not None and node.slog.open:
            node.slog.close()
            node.get_logger().warn("stopped before the scenario ended: %d rows written to %s (incomplete)" % (node.slog.n, node.slog.path))
        try:
            z = Float64MultiArray(); z.data = [0.0] * 12
            node.pub.publish(z)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
