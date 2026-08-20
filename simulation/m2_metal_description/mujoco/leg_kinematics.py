"""
Forward and inverse kinematics for the xTerra Svan M2 quadruped's legs.

Geometry (measured from m2_metal_description.urdf):
    D  = hip(abduction)-to-thigh joint offset = 0.0801 m
    L2 = thigh length (thigh joint to knee joint)  = 0.1915 m
    L3 = calf length (knee joint to foot contact)  = 0.19429 m

Convention:
    q1 = hip abduction angle   (rotation about local x)
    q2 = thigh/hip pitch angle (rotation about local y)
    q3 = knee/calf pitch angle (rotation about local y)
    Foot position (px, py, pz) is relative to the hip joint's attachment
    point on the body, in the body/hip-attachment frame.
    d = signed hip-to-thigh offset: +D for left legs (FL, RL), -D for right
    legs (FR, RR) -- the same formula covers all four legs via this sign.

Verified two ways:
  1. Round-tripping FK -> IK -> FK to machine precision for both signs of d.
  2. Cross-checked against MuJoCo's own forward kinematics on the actual
     xterra_m2 model (matches to ~1mm; residual is a small foot-pad mesh
     offset, not a modeling error).
"""
import numpy as np

D = 0.0801
L2 = 0.1915
L3 = 0.19429


def forward_kinematics(q1: float, q2: float, q3: float, d: float = -D, l2: float = L2, l3: float = L3):
    """
    Return foot position (px, py, pz) given joint angles.
    d: signed hip offset. Default -D = right-side leg (FR/RR) convention.
       Pass d=+D for left-side legs (FL/RL).
    """
    L = l2 * np.cos(q2) + l3 * np.cos(q2 + q3)
    px = -(l2 * np.sin(q2) + l3 * np.sin(q2 + q3))
    py = d * np.cos(q1) + L * np.sin(q1)
    pz = d * np.sin(q1) - L * np.cos(q1)
    return np.array([px, py, pz])


def inverse_kinematics(px: float, py: float, pz: float, d: float = -D, knee_sign: int = -1,
                        l2: float = L2, l3: float = L3):
    """
    Return joint angles (q1, q2, q3) for a desired foot position.
    d: signed hip offset, same convention as forward_kinematics (-D right, +D left).
    knee_sign = -1 matches Svan M2's natural backward-bending knee
    (its own home pose uses q3 = -1.8 rad); +1 is the other, physically
    awkward forward-bending branch.
    """
    L = np.sqrt(max(py ** 2 + pz ** 2 - d ** 2, 0.0))
    q1 = np.arctan2(L * py + d * pz, d * py - L * pz)

    x, y = -px, L
    c3 = np.clip((x ** 2 + y ** 2 - l2 ** 2 - l3 ** 2) / (2 * l2 * l3), -1.0, 1.0)
    q3 = knee_sign * np.arccos(c3)
    q2 = np.arctan2(x, y) - np.arctan2(l3 * np.sin(q3), l2 + l3 * np.cos(q3))
    return q1, q2, q3


if __name__ == "__main__":
    # Round-trip self-check for both right-side and left-side legs
    test_angles = [(0.0, 0.9, -1.8), (0.3, 0.5, -1.0), (-0.2, 0.2, -0.4)]
    for d, label in [(-D, "right (FR/RR)"), (D, "left (FL/RL)")]:
        for q1, q2, q3 in test_angles:
            p = forward_kinematics(q1, q2, q3, d=d)
            q1r, q2r, q3r = inverse_kinematics(*p, d=d)
            p_check = forward_kinematics(q1r, q2r, q3r, d=d)
            err = np.linalg.norm(p - p_check)
            print(f"{label:14s} q=({q1:+.3f},{q2:+.3f},{q3:+.3f})  "
                  f"recovered=({q1r:+.3f},{q2r:+.3f},{q3r:+.3f})  fk_err={err:.2e} m")
