#!/usr/bin/env python3
"""compact_urdf.py -- expand the xTerra xacro/URDF and apply the *simulation-only* overlay.

The files in urdf/ and meshes/ are NOT edited.  This script (called from gazebo.launch.py)
prints the URDF that Gazebo / robot_state_publisher receive, with these changes:

  1. collision geometry: the 17 STL-mesh <collision> elements are replaced by simple
     primitives: one box for the body, one sphere per foot, none for hip/thigh/calf links.
     Mesh-vs-mesh collision in ODE is very slow (your log shows real-time factor ~0.2) and
     makes foot contact jittery.  Visuals still use the xTerra meshes.   [--mesh-collisions keeps the originals]
  2. <dynamics damping> on the 12 actuated joints (same value as the xTerra MuJoCo model).
  3. ros2_control <state_interface name="position"> gets initial_value = standing pose, so
     gazebo_ros2_control puts the robot in the standing pose at spawn instead of straight legs.
     (gazebo_ros2_control calls SetPosition(initial_value) when it loads.)   [--no-initial-pose disables]
  4. <gazebo reference="*_foot"> friction / contact stiffness.
  5. A gazebo_ros_p3d plugin that publishes ground-truth base pose on /ground_truth/odom
     (simulation only -- on the real robot the pose comes from the state estimator).
  6. --zero-offset (fallback, instead of 3): joint angle 0 = stand pose (see zero_offset_pose)
  7. optional --manual-limits : joint limits from the Svan M2 manual (+-48 deg / -200..90 deg / -156..-48 deg).
  8. optional --lidar : a 2-D lidar on top of the body (link lidar_link, 0.83 kg, 360 beams, 10 Hz, 0.15-6 m, publishes /scan).
     This is a SIMULATION stand-in for xTerra's optional Velodyne LiDAR; the mounting position is a proposal until
     xTerra confirms theirs (see README_NAVIGATION.md).  The lidar link has a visual and a mass but NO collision geometry,
     so its own rays never hit the robot.
"""
import argparse
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import svan_core as C   # noqa: E402

JOINT_ORDER = [f"{l}_{j}_joint" for l in C.LEGS for j in ("hip", "thigh", "calf")]

# (shape, size tuple, origin xyz) measured from the STL bounding boxes (see README)
COLLISION = {
    "base_link": ("box", (0.4205, 0.2130, 0.1945), (0.0703, 0.0, 0.0245)),
    "foot":      ("sphere", (0.0172,), (0.0086, 0.0, 0.0135)),
}


# --lidar parameters: lidar_link origin in base_link [m], size and mass of a VLP-16-class puck (datasheet values quoted from memory -> verify)
LIDAR_XYZ = (0.05, 0.0, 0.158)       # 5 cm ahead of the base origin (about the middle of the body box), just above its top face (z = 0.1218)
LIDAR_RADIUS, LIDAR_LENGTH, LIDAR_MASS = 0.0515, 0.072, 0.83


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def prim_collision(shape, size, xyz):
    col = ET.Element("collision")
    ET.SubElement(col, "origin", xyz=fmt(xyz), rpy="0 0 0")
    geom = ET.SubElement(col, "geometry")
    if shape == "box":
        ET.SubElement(geom, "box", size=fmt(size))
    else:
        ET.SubElement(geom, "sphere", radius=fmt(size))
    return col


def replace_collisions(root):
    for link in root.findall("link"):
        name = link.get("name")
        cols = link.findall("collision")
        if not any(c.find("geometry/mesh") is not None for c in cols):
            continue
        for c in cols:
            link.remove(c)
        if name == "base_link":
            key = "base_link"
        elif name.endswith("_foot"):
            key = "foot"
        else:
            key = None                       # hip / thigh / calf links: no collision (only feet + body touch the ground)
        if key:
            link.append(prim_collision(*COLLISION[key]))


def set_dynamics(root, damping, friction):
    for j in root.findall("joint"):
        if j.get("name") in JOINT_ORDER:
            d = j.find("dynamics")
            if d is None:
                d = ET.SubElement(j, "dynamics")
            d.set("damping", str(damping))
            d.set("friction", str(friction))


def set_limits(root):
    rng = {"hip": (-math.radians(48), math.radians(48)),
           "thigh": (-math.radians(200), math.radians(90)),
           "calf": (-math.radians(156), -math.radians(48))}
    for j in root.findall("joint"):
        n = j.get("name")
        if n in JOINT_ORDER:
            lo, hi = rng[n.split("_")[1]]
            lim = j.find("limit")
            lim.set("lower", f"{lo:.4f}")
            lim.set("upper", f"{hi:.4f}")


def zero_offset_pose(root):
    """Alternative to initial_value: rotate every leg joint's origin by the stand angle so that joint angle 0
    *is* the stand pose.  /joint_states then reports (q_true - q_stand); svan_walk_controller (zero_offset:=true)
    adds q_stand back.  Needs no gazebo_ros2_control feature at all."""
    q = dict(zip(JOINT_ORDER, C.stand_joints()))
    for j in root.findall("joint"):
        n = j.get("name")
        if n not in JOINT_ORDER:
            continue
        o = j.find("origin")
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        assert rpy == [0.0, 0.0, 0.0], "unexpected joint origin rotation"
        axis = [float(v) for v in j.find("axis").get("xyz").split()]
        if axis == [1.0, 0.0, 0.0]:
            o.set("rpy", fmt((q[n], 0.0, 0.0)))
        elif axis == [0.0, 1.0, 0.0]:
            o.set("rpy", fmt((0.0, q[n], 0.0)))
        else:
            raise SystemExit(f"unsupported axis for {n}: {axis}")
        lim = j.find("limit")
        lim.set("lower", f"{float(lim.get('lower')) - q[n]:.6f}")
        lim.set("upper", f"{float(lim.get('upper')) - q[n]:.6f}")


def set_initial_pose(root):
    q = dict(zip(JOINT_ORDER, C.stand_joints()))
    rc = root.find("ros2_control")
    for jt in rc.findall("joint"):
        for si in jt.findall("state_interface"):
            if si.get("name") == "position":
                for p in si.findall("param"):
                    si.remove(p)
                ET.SubElement(si, "param", name="initial_value").text = f"{q[jt.get('name')]:.6f}"


def add_gazebo_blocks(root, mu, kp, kd):
    for l in C.LEGS:
        g = ET.SubElement(root, "gazebo", reference=f"{l}_foot")
        for tag, val in (("mu1", mu), ("mu2", mu), ("kp", kp), ("kd", kd), ("minDepth", 0.0005), ("maxVel", 1.0)):
            ET.SubElement(g, tag).text = str(val)
    # ground-truth pose.  After Gazebo's fixed-joint lumping the root link is called "base" (base_link is merged
    # into it); if your Gazebo keeps base_link as a link instead, the second plugin picks it up.  Whichever
    # link does not exist just prints one error at start-up and publishes nothing.
    for body in ("base", "base_link"):
        g = ET.SubElement(root, "gazebo")
        p = ET.SubElement(g, "plugin", name=f"p3d_ground_truth_{body}", filename="libgazebo_ros_p3d.so")
        ros = ET.SubElement(p, "ros")
        ET.SubElement(ros, "namespace").text = "/"
        ET.SubElement(ros, "remapping").text = "odom:=ground_truth/odom"
        for tag, val in (("body_name", body), ("frame_name", "world"), ("update_rate", "100"),
                         ("xyz_offset", "0 0 0"), ("rpy_offset", "0 0 0"), ("gaussian_noise", "0.0")):
            ET.SubElement(p, tag).text = val


def fmt1(v):
    return ("%.6f" % v).rstrip("0").rstrip(".") or "0"


def add_lidar(root, xyz=LIDAR_XYZ, rate=10.0, samples=360, rmin=0.15, rmax=6.0, noise=0.01):
    m, r, h = LIDAR_MASS, LIDAR_RADIUS, LIDAR_LENGTH
    ixx = m * (3 * r * r + h * h) / 12.0
    izz = 0.5 * m * r * r
    link = ET.SubElement(root, "link", name="lidar_link")
    inr = ET.SubElement(link, "inertial")
    ET.SubElement(inr, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inr, "mass", value=str(m))
    ET.SubElement(inr, "inertia", ixx="%.6g" % ixx, ixy="0", ixz="0", iyy="%.6g" % ixx, iyz="0", izz="%.6g" % izz)
    vis = ET.SubElement(link, "visual")
    ET.SubElement(vis, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(ET.SubElement(vis, "geometry"), "cylinder", radius=str(r), length=str(h))
    mat = ET.SubElement(vis, "material", name="lidar_black")
    ET.SubElement(mat, "color", rgba="0.08 0.08 0.08 1")
    j = ET.SubElement(root, "joint", name="lidar_joint", type="fixed")
    ET.SubElement(j, "parent", link="base_link")
    ET.SubElement(j, "child", link="lidar_link")
    ET.SubElement(j, "origin", xyz=" ".join(fmt1(v) for v in xyz), rpy="0 0 0")
    g = ET.SubElement(root, "gazebo", reference="lidar_link")
    s = ET.SubElement(g, "sensor", type="ray", name="svan_lidar")
    for tag, val in (("pose", "0 0 0 0 0 0"), ("visualize", "false"), ("update_rate", str(rate)), ("always_on", "true")):
        ET.SubElement(s, tag).text = val
    ray = ET.SubElement(s, "ray")
    hz = ET.SubElement(ET.SubElement(ray, "scan"), "horizontal")
    for tag, val in (("samples", str(samples)), ("resolution", "1"), ("min_angle", "-3.141593"), ("max_angle", "3.141593")):
        ET.SubElement(hz, tag).text = val
    rg = ET.SubElement(ray, "range")
    for tag, val in (("min", str(rmin)), ("max", str(rmax)), ("resolution", "0.01")):
        ET.SubElement(rg, tag).text = val
    nz = ET.SubElement(ray, "noise")
    for tag, val in (("type", "gaussian"), ("mean", "0.0"), ("stddev", str(noise))):
        ET.SubElement(nz, tag).text = val
    p = ET.SubElement(s, "plugin", name="svan_lidar_ros", filename="libgazebo_ros_ray_sensor.so")
    ros = ET.SubElement(p, "ros")
    ET.SubElement(ros, "namespace").text = "/"
    ET.SubElement(ros, "remapping").text = "~/out:=scan"
    ET.SubElement(p, "output_type").text = "sensor_msgs/LaserScan"
    ET.SubElement(p, "frame_name").text = "lidar_link"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--mesh-collisions", action="store_true", help="keep the original STL collision meshes")
    ap.add_argument("--no-initial-pose", action="store_true")
    ap.add_argument("--zero-offset", action="store_true", help="fallback: make joint angle 0 the stand pose (see zero_offset_pose)")
    ap.add_argument("--manual-limits", action="store_true")
    ap.add_argument("--lidar", action="store_true", help="add the simulated 2-D lidar (lidar_link + ray sensor plugin, publishes /scan)")
    ap.add_argument("--damping", type=float, default=1.0)
    ap.add_argument("--friction", type=float, default=0.0)
    ap.add_argument("--foot-mu", type=float, default=1.0)
    ap.add_argument("--foot-kp", type=float, default=1.0e6)
    ap.add_argument("--foot-kd", type=float, default=100.0)
    a = ap.parse_args()

    out = subprocess.run(["xacro", a.urdf], check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(out)
    if not a.mesh_collisions:
        replace_collisions(root)
    set_dynamics(root, a.damping, a.friction)
    if a.manual_limits:
        set_limits(root)
    if a.zero_offset:
        zero_offset_pose(root)
    elif not a.no_initial_pose:
        set_initial_pose(root)
    add_gazebo_blocks(root, a.foot_mu, a.foot_kp, a.foot_kd)
    if a.lidar:
        add_lidar(root)
    sys.stdout.write(ET.tostring(root, encoding="unicode"))


if __name__ == "__main__":
    main()
