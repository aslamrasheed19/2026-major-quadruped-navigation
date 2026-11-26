"""svan_core.py -- ROS-free core of the Svan M2 stand / trot / waypoint controller.

Everything here is plain Python + NumPy so the exact same code can be tested
offline (MuJoCo) and then imported by the ROS 2 node svan_walk_controller.py.

Joint order everywhere (same as controllers.yaml):
    FR_hip FR_thigh FR_calf  FL_hip FL_thigh FL_calf  RR_hip RR_thigh RR_calf  RL_hip RL_thigh RL_calf
Joint convention (from the URDF): hip = rotation about +x, thigh/calf = rotation about +y.
A positive thigh angle swings the foot BACKWARD, a negative calf angle folds the knee.
"""
import math
import numpy as np

LEGS = ["FR", "FL", "RR", "RL"]

# ---- geometry copied from m2_metal_description.urdf (joint origins) ------------------
HIP_XYZ = {"FR": (0.19085, -0.06, 0.00025), "FL": (0.19085, 0.06, 0.00025),
           "RR": (-0.19085, -0.06, 0.00025), "RL": (-0.19085, 0.06, 0.00025)}
THIGH_Y = {"FR": -0.0801, "FL": 0.0811, "RR": -0.0801, "RL": 0.0801}      # thigh joint y-offset
L2 = 0.1915                                                                # thigh length (calf joint z-offset)
FOOT_OFF = {"FR": (-0.00011504, -0.001, -0.20587), "FL": (-0.00011504, -0.0010179, -0.20587),
            "RR": (-0.00011504, -0.001, -0.20587), "RL": (-0.00011504, 0.00098207, -0.20587)}

TAU_LIM = np.array([11.92, 11.92, 23.84] * 4)      # Nm, from the xTerra manual / ros2_control block


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def foot_pos(leg, q3):
    """Exact forward kinematics: foot position in the BASE frame for one leg, q3=(hip,thigh,calf)."""
    h = np.array(HIP_XYZ[leg])
    inner = np.array([0.0, THIGH_Y[leg], 0.0]) + _ry(q3[1]) @ (np.array([0, 0, -L2]) + _ry(q3[2]) @ np.array(FOOT_OFF[leg]))
    return h + _rx(q3[0]) @ inner


def foot_jac(leg, q3, eps=1e-6):
    J = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3); d[i] = eps
        J[:, i] = (foot_pos(leg, q3 + d) - foot_pos(leg, q3 - d)) / (2 * eps)
    return J


def leg_ik(leg, p, q_init=None, iters=40):
    """Numerical IK (Newton) on the exact FK; seeded near the standing 'knee-back' posture."""
    q = np.array([0.0, 0.9, -1.8]) if q_init is None else np.array(q_init, float)
    for _ in range(iters):
        e = np.asarray(p) - foot_pos(leg, q)
        if np.linalg.norm(e) < 1e-9:
            break
        J = foot_jac(leg, q)
        q = q + np.linalg.lstsq(J, e, rcond=1e-6)[0] * 0.8
    return q


# ---- balanced stand ------------------------------------------------------------------
# The whole-robot CoM sits ~5.6 cm in front of the hip centre, so with the feet directly under the
# hips the front legs carry ~62 % of the weight.  Shifting all four feet forward by FOOT_SHIFT_X
# (computed offline with Pinocchio so that the CoM is in the middle of the support polygon)
# equalises the load.  See make_stand() / the README.
FOOT_SHIFT_X = 0.047
STAND_HEIGHT = 0.250          # hip-to-ground height in metres (xTerra default pose is 0.247)


def nominal_feet(shift_x=FOOT_SHIFT_X, height=STAND_HEIGHT):
    """Nominal foot positions (base frame) used both for standing and as the centre of the trot."""
    out = {}
    for l in LEGS:
        h = np.array(HIP_XYZ[l]); fo = np.array(FOOT_OFF[l])
        # lateral position straight below the hip + thigh offset, forward shift applied on x
        y = h[1] + THIGH_Y[l] + fo[1]
        out[l] = np.array([h[0] + 0.011 + shift_x, y, -height])
    return out


def stand_joints(shift_x=FOOT_SHIFT_X, height=STAND_HEIGHT):
    feet = nominal_feet(shift_x, height)
    q = []
    for l in LEGS:
        q.extend(leg_ik(l, feet[l]))
    return np.array(q)


# ---- fast analytic IK (used in the control loop; ~10 microseconds) ---------------------
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def leg_ik_fast(leg, p):
    """Closed-form IK for the exact URDF geometry.  Knee bends 'backwards' (calf angle < 0)."""
    h = HIP_XYZ[leg]
    wx, wy, wz = p[0] - h[0], p[1] - h[1], p[2] - h[2]
    d = THIGH_Y[leg] + FOOT_OFF[leg][1]                 # lateral offset of the leg plane
    fx, fz = FOOT_OFF[leg][0], FOOT_OFF[leg][2]
    L3 = math.hypot(fx, fz)
    delta = math.atan2(fx, -fz)
    r = math.sqrt(max(wy * wy + wz * wz - d * d, 1e-9))
    q1 = _wrap(math.atan2(wz, wy) - math.atan2(-r, d))
    px, pz = wx, -r
    D2 = px * px + pz * pz
    c = max(-1.0, min(1.0, (D2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)))
    dl = -math.acos(c)
    alpha = math.atan2(-px, -pz)
    q2 = alpha - math.atan2(L3 * math.sin(dl), L2 + L3 * math.cos(dl))
    return np.array([q1, q2, dl + delta])


def stand_joints_fast(shift_x=FOOT_SHIFT_X, height=STAND_HEIGHT):
    f = nominal_feet(shift_x, height)
    return np.concatenate([leg_ik_fast(l, f[l]) for l in LEGS])


# ---- steady-state torques needed to hold the stand pose (computed by tools/compute_ff.py) --
# order: FR FL RR RL x (hip, thigh, calf).  Filled in below after calibration.
FF_STAND = np.array([1.476, 1.849, 5.413, -1.476, 1.849, 5.413, 1.296, 1.719, 5.006, -1.296, 1.719, 5.006])


def smoothstep(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


class SvanController:
    """Joint-space PD + feed-forward controller that stands, trots and turns.

    step(t, q, dq) -> 12 joint torques [Nm].  Call it at the control rate (500 Hz).
    command(vx, vy, wz) sets the desired body velocity (m/s, m/s, rad/s).
    """

    def __init__(self, kp=(60.0, 80.0, 90.0), kd=(0.3, 0.36, 0.45), ki=0.0,
                 ramp_time=3.0, period=0.50, duty=0.50, step_height=0.05,
                 ff_scale=1.0, shift_x=FOOT_SHIFT_X, height=STAND_HEIGHT,
                 max_acc=0.4, max_wacc=1.5, v_limit=0.35, w_limit=1.0):
        self.kp = np.tile(np.array(kp, float), 4)
        self.kd = np.tile(np.array(kd, float), 4)
        self.ki = ki
        self.ramp_time, self.T, self.beta, self.h_step = ramp_time, period, duty, step_height
        self.ff_scale = ff_scale
        self.nom = nominal_feet(shift_x, height)
        self.q_stand = np.concatenate([leg_ik_fast(l, self.nom[l]) for l in LEGS])
        self.offsets = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}          # diagonal pairs
        self.max_acc, self.max_wacc, self.v_limit, self.w_limit = max_acc, max_wacc, v_limit, w_limit
        self.cmd = np.zeros(3)          # requested vx, vy, wz
        self.vf = np.zeros(3)           # rate-limited
        self.s = 0.0                    # gait amplitude 0..1
        self.phase = 0.0
        self.q0 = None; self.t0 = None; self.t_prev = None
        self.q_ref_prev = None
        self.integ = np.zeros(12)
        self.mode = "ramp"

    # ---- API ------------------------------------------------------------------------
    def start(self, t, q_measured):
        """(Re)start the stand-up ramp from the measured joint angles; also clears the gait state."""
        self.q0 = np.array(q_measured, float); self.t0 = t; self.t_prev = t
        self.q_ref_prev = self.q0.copy(); self.mode = "ramp"; self.integ[:] = 0
        self.cmd[:] = 0; self.vf[:] = 0; self.s = 0.0; self.phase = 0.0

    def command(self, vx=0.0, vy=0.0, wz=0.0):
        self.cmd = np.array([np.clip(vx, -self.v_limit, self.v_limit), np.clip(vy, -self.v_limit, self.v_limit),
                             np.clip(wz, -self.w_limit, self.w_limit)])

    # ---- foot trajectory --------------------------------------------------------------
    def _foot_targets(self):
        feet, sw = {}, {}
        vx, vy, wz = self.vf
        for l in LEGS:
            n = self.nom[l]
            ph = (self.phase + self.offsets[l]) % 1.0
            vleg = np.array([vx - wz * n[1], vy + wz * n[0]])         # foot velocity needed in body frame
            stride = vleg * self.T * self.beta                          # displacement during stance
            if ph < self.beta:                                          # stance: foot slides backwards
                u = ph / self.beta
                off = 0.5 * stride * (1 - 2 * u); dz = 0.0; swing = 0.0
            else:                                                       # swing: foot goes forward and up
                u = (ph - self.beta) / (1 - self.beta)
                off = 0.5 * stride * (-1 + 2 * smoothstep(u))
                dz = self.s * self.h_step * 0.5 * (1 - math.cos(2 * math.pi * u)); swing = 1.0
            feet[l] = np.array([n[0] + off[0], n[1] + off[1], n[2] + dz])
            # smooth contact weight (for load sharing of the feed-forward torque)
            edge = min(ph - self.beta if ph >= self.beta else 1.0, (1.0 - ph) if ph >= self.beta else 1.0)
            sw[l] = self.s * swing * smoothstep(edge / 0.08)
        return feet, sw

    # ---- main step -----------------------------------------------------------------
    def step(self, t, q, dq):
        q = np.asarray(q, float); dq = np.asarray(dq, float)
        dt = t - self.t_prev
        if not (1e-5 < dt < 0.05):
            dt = 0.002
        self.t_prev = t
        tt = t - self.t0
        ff = np.zeros(12)
        if tt < self.ramp_time:                                     # sit/limp pose -> stand, from the MEASURED pose
            a = smoothstep(tt / self.ramp_time)
            da = (6 * (tt / self.ramp_time) * (1 - tt / self.ramp_time)) / self.ramp_time
            q_ref = self.q0 + a * (self.q_stand - self.q0)
            dq_ref = da * (self.q_stand - self.q0)
            # started far from the stand pose (e.g. straight legs): bring the gravity feed-forward in gradually
            far = float(np.max(np.abs(self.q0 - self.q_stand))) > 0.3
            ff = self.ff_scale * FF_STAND * (a if far else 1.0)
        else:
            self.mode = "walk" if (self.s > 1e-3 or np.any(self.cmd != 0)) else "stand"
            # rate-limit velocity command
            for i, acc in enumerate((self.max_acc, self.max_acc, self.max_wacc)):
                self.vf[i] += np.clip(self.cmd[i] - self.vf[i], -acc * dt, acc * dt)
            moving = np.any(np.abs(self.cmd) > 1e-6) or np.any(np.abs(self.vf) > 1e-3)
            self.s += np.clip((1.0 if moving else 0.0) - self.s, -dt / 0.5, dt / 0.5)
            if self.s > 1e-3:
                self.phase = (self.phase + dt / self.T) % 1.0
            feet, sw = self._foot_targets()
            q_ref = np.concatenate([leg_ik_fast(l, feet[l]) for l in LEGS])
            dq_ref = (q_ref - self.q_ref_prev) / dt if self.q_ref_prev is not None else np.zeros(12)
            wgt = np.array([1.0 - sw[l] for l in LEGS])
            share = 4.0 * wgt / max(wgt.sum(), 1e-6)
            ff = self.ff_scale * np.concatenate([share[i] * FF_STAND[3 * i:3 * i + 3] for i in range(4)])
        self.q_ref_prev = q_ref
        e = q_ref - q
        if self.ki > 0:
            self.integ = np.clip(self.integ + e * dt, -1.0, 1.0)
        tau = ff + self.kp * e + self.ki * self.integ + self.kd * (dq_ref - dq)
        return np.clip(tau, -TAU_LIM, TAU_LIM), q_ref


class FlipMonitor:
    """Tells whether the body is lying on its side / upside down.

    update(t, tilt) is called with the simulation time and the tilt angle of the body [rad] (angle between the
    body z axis and the world z axis, 0 = upright, pi = upside down).  It returns
        "flipped"   once, when the tilt has been above `limit` for `hold` seconds,
        "recovered" once, when the body has been below `limit` for `ok_hold` seconds after a flip,
        None        otherwise.
    Normal trotting has a tilt of a few degrees, so the default limit of 1.0 rad (57 deg) never triggers.
    """

    def __init__(self, limit=1.0, hold=1.0, ok_hold=0.5):
        self.limit, self.hold, self.ok_hold = limit, hold, ok_hold
        self.flipped = False
        self._since = None            # time when the current "bad" (or, while flipped, "good") streak started

    def reset(self):
        self.flipped = False; self._since = None

    def update(self, t, tilt):
        bad = tilt > self.limit
        if not self.flipped:
            if not bad:
                self._since = None
                return None
            if self._since is None:
                self._since = t
            if t - self._since >= self.hold:
                self.flipped = True; self._since = None
                return "flipped"
            return None
        if bad:
            self._since = None
            return None
        if self._since is None:
            self._since = t
        if t - self._since >= self.ok_hold:
            self.flipped = False; self._since = None
            return "recovered"
        return None


class WaypointFollower:
    """Heading + distance controller that turns a list of (x, y) waypoints into (vx, wz)."""

    def __init__(self, waypoints, v_max=0.25, w_max=0.8, k_head=1.5, reach=0.08, turn_thresh=0.6, slow_dist=0.4):
        self.wps = [tuple(map(float, w)) for w in waypoints]
        self.i = 0
        self.v_max, self.w_max, self.k, self.reach, self.turn_thresh, self.slow = v_max, w_max, k_head, reach, turn_thresh, slow_dist

    @property
    def done(self):
        return self.i >= len(self.wps)

    def update(self, x, y, yaw):
        while not self.done:
            dx, dy = self.wps[self.i][0] - x, self.wps[self.i][1] - y
            dist = math.hypot(dx, dy)
            if dist < self.reach:
                self.i += 1
            else:
                break
        if self.done:
            return 0.0, 0.0
        err = _wrap(math.atan2(dy, dx) - yaw)
        wz = float(np.clip(self.k * err, -self.w_max, self.w_max))
        if abs(err) > self.turn_thresh:
            return 0.0, wz                                            # turn on the spot first
        vx = self.v_max * min(1.0, dist / self.slow) * max(0.0, math.cos(err))
        return float(max(vx, 0.08 if dist > self.reach else 0.0)), wz
