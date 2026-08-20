"""
Record a clean MP4 of the Svan M2 walking gait directly from MuJoCo's
offscreen renderer. No screen-capture artifacts, no phone-filming-a-laptop
quality loss, exact resolution/framing every time -- this is the right way
to produce demo footage for a report or README, not a phone video.

Requires: pip install imageio[ffmpeg]

Usage:
    python3 record_walk.py scene.xml walk_demo.mp4 --duration 6
"""
import argparse
import numpy as np
import mujoco
import imageio

from walk_demo import LEGS, D, D_SIGN, leg_fk, leg_ik, foot_target, KP, KD


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", nargs="?", default="scene.xml")
    parser.add_argument("output", nargs="?", default="walk_demo.mp4")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    joint_qadr, joint_vadr, act_id = {}, {}, {}
    for leg in LEGS:
        for part in ["hip", "thigh", "calf"]:
            name = f"{leg}_{part}_joint"
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            joint_qadr[name] = model.jnt_qposadr[jid]
            joint_vadr[name] = model.jnt_dofadr[jid]
            act_id[f"{leg}_{part}"] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg}_{part}")

    p0 = leg_fk(0.0, 0.9, -1.8, -D)
    p0x, p0z = p0[0], p0[2]
    nominal_height = data.qpos[2]  # standing height at the home keyframe, used to keep the camera's vertical aim steady

    renderer = mujoco.Renderer(model, height=480, width=640)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance = 1.2
    cam.elevation = -15
    cam.azimuth = 120

    n_steps = int(args.duration / model.opt.timestep)
    frame_every = max(1, round(1.0 / (args.fps * model.opt.timestep)))
    frames = []

    for step in range(n_steps):
        t = data.time
        for leg in LEGS:
            target = foot_target(leg, t, p0x, p0z)
            q1, q2, q3 = leg_ik(*target, D_SIGN[leg])
            for part, qtarget in zip(["hip", "thigh", "calf"], [q1, q2, q3]):
                jname = f"{leg}_{part}_joint"
                qa, va = joint_qadr[jname], joint_vadr[jname]
                data.ctrl[act_id[f"{leg}_{part}"]] = KP * (qtarget - data.qpos[qa]) - KD * data.qvel[va]
        mujoco.mj_step(model, data)

        if step % frame_every == 0:
            # Camera fix: only pan horizontally (x, y). Height is held at a
            # fixed nominal value instead of following data.qpos[2] raw --
            # the base bounces ~3cm every stride (normal gait behavior), and
            # blindly copying that into the camera's lookat made the whole
            # viewport bob at the stride frequency, reading as "shaky camera"
            # even though the walk itself is smooth. The robot's own bounce
            # is still fully visible in-frame; only the *viewpoint* is now
            # steady.
            cam.lookat[0] = data.qpos[0]
            cam.lookat[1] = data.qpos[1]
            cam.lookat[2] = nominal_height
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    imageio.mimwrite(args.output, frames, fps=args.fps, quality=8)
    print(f"Wrote {len(frames)} frames ({args.duration:.1f}s @ {args.fps}fps) to {args.output}")


if __name__ == "__main__":
    main()
