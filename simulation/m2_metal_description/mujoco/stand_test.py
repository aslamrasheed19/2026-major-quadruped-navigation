"""
Standing-pose validation for the xTerra Svan M2 quadruped (MuJoCo).

Loads the model's home keyframe and holds it with a simple joint-space PD
controller for a few seconds, confirming the URDF/MJCF (masses, inertias,
joint limits, collision meshes) is physically stable before building any
real controller on top of it.

Usage:
    python3 stand_test.py /path/to/xterra_m2/m2_metal_description/mujoco/scene.xml
"""
import sys
import numpy as np
import mujoco


def run_stand_test(scene_path: str, kp: float = 60.0, kd: float = 2.0, sim_seconds: float = 2.0):
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)

    qpos_home = model.key_qpos[0][7:].copy()  # target joint angles (skip 7 floating-base DOF)
    n_steps = int(sim_seconds / model.opt.timestep)

    for _ in range(n_steps):
        q, qd = data.qpos[7:], data.qvel[6:]
        data.ctrl[:] = kp * (qpos_home - q) - kd * qd
        mujoco.mj_step(model, data)

    base_height = data.qpos[2]
    fell_over = base_height < 0.10 or np.isnan(data.qpos).any()

    print(f"Simulated {sim_seconds:.1f}s ({n_steps} steps)")
    print(f"Final base height: {base_height:.4f} m")
    print(f"Base position (x, y, z): {np.round(data.qpos[0:3], 4)}")
    print("Result:", "FAILED (fell over / NaN)" if fell_over else "PASSED (standing stable)")
    return not fell_over


if __name__ == "__main__":
    scene = sys.argv[1] if len(sys.argv) > 1 else "external/xterra_m2/m2_metal_description/mujoco/scene.xml"
    ok = run_stand_test(scene)
    sys.exit(0 if ok else 1)
