"""
Live standing demo for screen recording — Svan M2 in MuJoCo.

Opens the interactive viewer AND actively holds the home pose with a
joint-space PD controller in the same loop. This is necessary because the
model's actuators are direct-torque motors (not position servos), and the
keyframe's stored ctrl values are not real gravity-compensating torques —
without an active controller the robot collapses under gravity.

Usage:
    python3 stand_demo.py /path/to/xterra_m2/m2_metal_description/mujoco/scene.xml

Controls in the viewer window: mouse-drag to orbit, ctrl+right-click-drag
to push on the robot (nice for showing it's actually being held up
dynamically, not just posed). Close the window to stop.
"""
import sys
import time
import mujoco
import mujoco.viewer


def main(scene_path: str, kp: float = 60.0, kd: float = 2.0):
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    qpos_home = model.key_qpos[0][7:].copy()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            q, qd = data.qpos[7:], data.qvel[6:]
            data.ctrl[:] = kp * (qpos_home - q) - kd * qd
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    scene = sys.argv[1] if len(sys.argv) > 1 else "external/xterra_m2/m2_metal_description/mujoco/scene.xml"
    main(scene)
