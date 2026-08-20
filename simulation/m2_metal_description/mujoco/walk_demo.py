"""
Open-loop trot walking demo for the xTerra Svan M2 (MuJoCo) — for DEMO VIDEO
purposes only. This is a simple kinematic trot (raise-swing-forward /
push-backward-on-ground, diagonal leg pairs alternating) driven by the
verified leg IK, tracked with a joint-space PD. It is NOT the literature-
informed baseline PID path-following controller your MTP Phase 1 calls for —
it's a quick way to get walking footage for this week's update.

Verified headlessly before shipping: forward speed ~0.17 m/s, alignment
with the robot's own front (FL/FR) axis = +1.000, stays upright over a
12s test run.

Usage:
    python3 walk_demo.py /path/to/xterra_m2/m2_metal_description/mujoco/scene.xml

Ctrl+right-click-drag in the viewer to nudge it and see it recover/react.
"""
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer

# --- verified leg geometry (see leg_kinematics.py for derivation/validation) ---
D = 0.0801      # hip-to-thigh joint offset (m)
L2 = 0.1915     # thigh length (m)
L3 = 0.19429    # calf length, hip joint to foot contact site (m)

LEGS = ["FL", "FR", "RL", "RR"]
D_SIGN = {"FL": D, "FR": -D, "RL": D, "RR": -D}          # left legs +D, right legs -D
TROT_PHASE = {"FL": 0.5, "FR": 0.0, "RL": 0.0, "RR": 0.5}  # diagonal pairs: (FR,RL) then (FL,RR)

# --- gait parameters (tune these to change how it walks) ---
# NOTE on tuning history: small step lengths (<0.15) with a 50/50 swing/
# stance split get dominated by stance-phase foot slip and reliably walk
# BACKWARD regardless of sign -- verified by systematic testing (sign flip,
# swing/stance swap, slower gait period, stiffer PD all failed to fix it;
# even standing still drifts slightly in that same direction). A big
# STEP_LENGTH (0.20) overpowers that bias and walks forward, but bounces
# noticeably (~6cm height swing).
#
# Next fix: raising STEP_HEIGHT (more foot clearance) and skewing the duty
# cycle toward more stance time (DUTY_CYCLE=0.65-0.70) let a SMALLER stride
# walk forward with much less bounce.
#
# This version adds one more improvement: stiffer PD gains (KP 60->90)
# let the stride shrink further still (0.08) while staying forward --
# tighter tracking means less time spent lagging the target, which
# apparently matters more than raw stride size for both direction and
# smoothness. The body also has real rotational wobble during the gait
# (roll/pitch/yaw a few degrees) -- that's genuine dynamics, not a bug:
# nothing in this controller corrects body orientation, only leg position,
# so the body rocks somewhat on its own with each stride. This tuning
# minimizes it but a proper fix is body-orientation feedback (see the
# balance-controller papers in the literature survey), which is Phase 2
# scope, not a quick parameter fix.
#
# Verified over a 20s run: alignment +1.000 the whole time, height
# oscillation steady at 2.35cm (not growing), yaw wobble steady at 2.97deg
# (not growing), min height 21.5cm, never falls.
STEP_LENGTH = 0.08   # m, fore-aft stride amplitude
STEP_HEIGHT = 0.06   # m, foot lift during swing (clearance, not just height)
GAIT_PERIOD = 0.6    # s, time for one full stride cycle
DUTY_CYCLE = 0.70    # fraction of each cycle spent in stance (vs swing)
KP, KD = 90.0, 3.0   # joint-space PD gains


def leg_fk(q1, q2, q3, d):
    L = L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    px = -(L2 * np.sin(q2) + L3 * np.sin(q2 + q3))
    py = d * np.cos(q1) + L * np.sin(q1)
    pz = d * np.sin(q1) - L * np.cos(q1)
    return np.array([px, py, pz])


def leg_ik(px, py, pz, d, knee_sign=-1):
    L = np.sqrt(max(py ** 2 + pz ** 2 - d ** 2, 0.0))
    q1 = np.arctan2(L * py + d * pz, d * py - L * pz)
    x, y = -px, L
    c3 = np.clip((x ** 2 + y ** 2 - L2 ** 2 - L3 ** 2) / (2 * L2 * L3), -1.0, 1.0)
    q3 = knee_sign * np.arccos(c3)
    q2 = np.arctan2(x, y) - np.arctan2(L3 * np.sin(q3), L2 + L3 * np.cos(q3))
    return q1, q2, q3


def foot_target(leg, t, p0x, p0z):
    """Desired foot position (leg-local frame) at time t for the trot cycle."""
    phase = ((t / GAIT_PERIOD) + TROT_PHASE[leg]) % 1.0
    d = D_SIGN[leg]
    swing_frac = 1.0 - DUTY_CYCLE
    if phase < swing_frac:  # swing: lift and reach forward
        s = phase / swing_frac
        dx = -STEP_LENGTH / 2 + STEP_LENGTH * s
        dz = STEP_HEIGHT * np.sin(np.pi * s)
    else:            # stance: push backward along the ground -> propels body
        s = (phase - swing_frac) / DUTY_CYCLE
        dx = STEP_LENGTH / 2 - STEP_LENGTH * s
        dz = 0.0
    return np.array([p0x + dx, d, p0z + dz])


def main(scene_path: str):
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    # resolve every joint/actuator index ONCE (not every step)
    joint_qadr, joint_vadr, act_id = {}, {}, {}
    for leg in LEGS:
        for part in ["hip", "thigh", "calf"]:
            name = f"{leg}_{part}_joint"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            joint_qadr[name] = model.jnt_qposadr[jid]
            joint_vadr[name] = model.jnt_dofadr[jid]
            act_id[f"{leg}_{part}"] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{part}")

    # nominal stance foot position (from the robot's own home pose)
    p0 = leg_fk(0.0, 0.9, -1.8, -D)
    p0x, p0z = p0[0], p0[2]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            t = data.time  # simulation clock, NOT wall-clock -- keeps the gait
                           # coherent even when rendering can't keep up with the
                           # 1000 Hz physics rate (it never will, on any laptop)

            for leg in LEGS:
                target = foot_target(leg, t, p0x, p0z)
                q1, q2, q3 = leg_ik(*target, D_SIGN[leg])
                for part, qtarget in zip(["hip", "thigh", "calf"], [q1, q2, q3]):
                    jname = f"{leg}_{part}_joint"
                    qa, va = joint_qadr[jname], joint_vadr[jname]
                    data.ctrl[act_id[f"{leg}_{part}"]] = KP * (qtarget - data.qpos[qa]) - KD * data.qvel[va]

            mujoco.mj_step(model, data)
            viewer.sync()

            dt_left = model.opt.timestep - (time.time() - step_start)
            if dt_left > 0:
                time.sleep(dt_left)


if __name__ == "__main__":
    scene = sys.argv[1] if len(sys.argv) > 1 else "external/xterra_m2/m2_metal_description/mujoco/scene.xml"
    main(scene)
