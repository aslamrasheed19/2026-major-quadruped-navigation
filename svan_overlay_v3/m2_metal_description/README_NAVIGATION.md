# Svan M2 – obstacle avoidance overlay (v2)

Adds, on top of the working stand + trot + waypoint overlay:

1. **`nav:=known`** – obstacles are in the Gazebo world **and** given to the robot; it plans around them (A\* on a grid) and walks the detour.
2. **`nav:=sensed`** – the same obstacles are in the world but the robot is **not told** where they are. A simulated 2-D lidar is mounted on the body; the robot builds an obstacle map from `/scan` while walking and re-plans whenever the map shows its path is blocked.
3. A feasibility note + mount proposal for the lidar, and a draft email to xTerra (end of this file).

> **Status in one line:** everything below was tested **offline** (MuJoCo physics built from your URDF + the real controller/planner/node code). **Nothing here has been run in Gazebo or on ROS 2 yet** – first run on your machine is the real test. See "What is and is not verified".

## Files (copy over your package exactly like the first overlay)

| file | new / changed |
|---|---|
| `scripts/svan_nav.py` | **new** – ROS-free: occupancy grid, A\* planner, path follower, lidar mapper, forward guard, `Navigator` |
| `scripts/svan_walk_controller.py` | changed – `nav_mode`, obstacle parameters, `/scan` subscriber, 20 Hz nav timer on its own callback group, publishes `/svan/path`, `/svan/obstacle_map`, TF `world→base` |
| `scripts/compact_urdf.py` | changed – new `--lidar` flag (adds `lidar_link`, 0.83 kg, ray sensor → `/scan`) |
| `launch/gazebo.launch.py` | changed – args `nav`, `lidar`, `nav_config`; builds the obstacle world from `config/nav.yaml` |
| `config/nav.yaml` | **new** – 2 m square + 3 obstacles (edit freely) |
| `CMakeLists.txt`, `package.xml` | install `svan_nav.py`, add `tf2_ros`, `python3-yaml`, `rcl_interfaces` |
| `offline_tests/` | **new** – the MuJoCo test rig (not installed into ROS) |

`nav:=none` (the default) behaves **exactly as before** (1 m square from `walk.yaml`, obstacles ignored).

## Run

```bash
cd ~/ros2_ws                         # your workspace root
colcon build --symlink-install --packages-select xterra_m2_description
source install/setup.bash

# step 1 – obstacles known beforehand
ros2 launch xterra_m2_description gazebo.launch.py nav:=known

# step 2 – obstacles unknown, lidar only
ros2 launch xterra_m2_description gazebo.launch.py nav:=sensed

# your own layout (same format as config/nav.yaml)
ros2 launch xterra_m2_description gazebo.launch.py nav:=sensed nav_config:=$HOME/my_nav.yaml
# lidar present but no navigation (to look at /scan):
ros2 launch xterra_m2_description gazebo.launch.py lidar:=true
```

The launch prints one line `nav:=sensed lidar=True 1 cylinder(s) + 2 box(es) from ... -> /tmp/svan_world_nav.world`. The obstacles are orange; the path is the 2 m square `(2,0) → (2,2) → (0,2) → (0,0)` with a box on the first and third legs and a cylinder on the second.

**What the terminal should show** (`nav:=sensed`):
```
nav_mode=sensed: no obstacle is given; the map is built from /scan
nav: plan 1: first plan -> waypoint 0, 2 points, 2.01 m          <- straight line, nothing seen yet
nav: plan 2: path blocked -> waypoint 0, 4 points, 2.49 m        <- the box appeared in the scans
nav: waypoint 0 reached at (1.95, -0.06)
...
nav: waypoint 3 reached at (0.00, 0.08)
t= 90.1s  nav:done  ...
```
`path blocked` re-plans are normal in `sensed` mode (that is the robot discovering obstacles). Dozens of them in a row are not – see Troubleshooting.

**RViz** (optional): `ros2 run rviz2 rviz2`, Fixed Frame `world`, add `Map` (`/svan/obstacle_map`), `Path` (`/svan/path`), `LaserScan` (`/scan`), `RobotModel`. Grey/black cells = what the robot believes; in `sensed` mode unseen cells are "unknown". (The `world→base` TF comes from the ground-truth odometry, i.e. simulation only.)

Teleop still overrides everything while keys are pressed (`/cmd_vel`).

## How it works (short)

* **Planner.** 5 cm grid. A cell is blocked if it is closer than **0.42 m** to an obstacle (robot radius 0.32 m – a circle around the body *and* legs, max robot size is 46.8 × 43.6 cm – plus 0.10 m margin). Extra cost near obstacles keeps the path off the edge. A\* + line-of-sight smoothing. Goal inside a blocked zone → moved to the nearest free cell.
* **Follower.** Aims at a point 0.30 m ahead on the path; turns on the spot when the heading error is > 0.6 rad (same rule as before). Output is the same `(vx, wz)` command the trot controller already accepts – **the gait/torque code is untouched**.
* **Mapper (`sensed`).** Each scan updates a log-odds grid: cells along a beam get "freer", the cell at the measured range gets "more occupied". A cell becomes an obstacle after ~2 hits, so a single noisy return is forgotten. Scans are paired with the odometry sample **nearest the scan stamp** (the robot turns during the scan delay). While the body is tilted, returns beyond ~0.85·h/sin(tilt) are ignored (a level beam from a tilted body hits the *floor* there – this removed phantom walls in the tests); above 11° nothing is mapped.
* **Replanning** when: the path crosses a newly seen obstacle (with hysteresis so noise at the cell edge does not make it flap), the robot is > 0.5 m off the path, or it made no progress for 4 s.
* **Forward guard.** Independent of the map: if the raw scan shows something within ~0.45 m straight ahead the speed is scaled down, to 0 at 0.10 m from the front of the body.
* **Safety.** If the nav thread stalls for > 0.5 s the torque loop commands zero velocity (stand) instead of the last command.
* **Payload.** With the lidar the node scales the gravity feed-forward by (12.659 + 0.83) / 12.659 = 1.066 (`payload_kg`, set by the launch file).

## What is and is not verified

**Verified offline** (MuJoCo, 1 ms step, masses/inertias/joint limits/foot spheres/body box from your URDF, the real `SvanController` at 500 Hz with one tick of latency, ground-truth pose, lidar by ray casting from the tilting body):

| test | result |
|---|---|
| control: same course, no avoidance | drives into the box (gap −3 mm) |
| default scenario, known / lidar-only | both reach all 4 waypoints, min gap to obstacles 0.26 m / 0.29 m |
| 6 designed cases (long wall, U-trap, 0.70 m gap, 1.0 m gap, obstacle on a waypoint, 7 obstacles) × known + lidar-only | 12 / 12 reached the goal, no contact, min gap 0.157 m |
| 8 random layouts × (known, lidar-only, lidar 2 m range, lidar noise 5 cm @ 5 Hz) | 32 / 32 reached the goal, no contact, min gap 0.205 m, ≤ 9 re-plans, body tilt ≤ 5.7° |
| ROS node file driven through a stand-in for rclpy (50 ms scan delay) | known and lidar-only both finish; node logic, parameters, map/path publishing run |
| `compact_urdf.py --lidar` | generates a valid URDF (MuJoCo loads it); sensor block is plain Gazebo-classic XML |

**NOT verified – please treat as untested until your first run:**

* Gazebo itself: `libgazebo_ros_ray_sensor.so` actually publishing `/scan` with the pose/frame from the lumped `lidar_link`; ODE contact with the obstacles; how Gazebo's real walking speed (your Gazebo log needs up to ~80 s for the 1 m square, MuJoCo ~32 s) interacts with the 4 s "no progress" timer.
* ROS 2 middleware: the multithreaded executor / callback groups, QoS matching of `/scan`, the launch `OpaqueFunction`, the `ParameterDescriptor(dynamic_typing=True)` default lists (there is a fallback if your rclpy rejects it), `tf2_ros` import.
* Everything on hardware. MuJoCo walks differently from Gazebo (contact model); the *distances* from the tests are meaningful, the *timings* are not.
* The lidar mass/size (0.83 kg, Ø103 × 72 mm) are VLP-16-class values from memory, **not** xTerra's actual part.
* Only a single horizontal scan plane is simulated. Obstacles must be taller than the plane (≈ 0.41 m above ground; the test obstacles are 0.6 m). A low box (< 0.4 m) is invisible to this sensor model – a real 16-layer lidar would see it at close range, a depth camera would too (below).

## Lidar: feasible? where?

**Feasible – yes, on paper.** From the brochure page you shared: the Svan M2 lists an optional RGB-depth camera, a Velodyne LiDAR and a Jetson-style compute module with "mounting options", it says the sensor suite is **not included in the base robot**, and the payload is up to **5 kg** – a ~0.8 kg lidar is well inside that. The robot's own computer is a Raspberry Pi 4B, which is **not** enough for 3-D lidar processing or SLAM, so a compute module is needed for hardware.

**Manufacturer-recommended position – I could not find one.** Nothing public (brochure, manual, GitHub) states where xTerra mounts the lidar/camera. That has to come from xTerra – the draft email is below. Until they answer, the simulation uses **my proposal**:

> `lidar_link` at **(+0.05, 0, +0.158) m in `base_link`** – on the flat top of the body, 5 cm ahead of the body origin, centred left-right, with the puck's bottom sitting on the body top face (top face z = 0.122 m).

Why there: (a) it is the highest flat point, so the 360° plane is not blocked by the body or the legs (in the MuJoCo trot the hip, knee and foot points never rise above the hip axis, 0.15 m below the plane); (b) it is almost exactly over the centre of mass (CoM x ≈ +0.056 m), so 0.83 kg there does not shift the front/rear leg loading – the stand-pose feed-forward only needs the 6.6 % scale-up; (c) it is the centre of body roll/pitch, so the sensor moves least when the body tilts (≈ 5° during the trot); (d) nothing else is known to be on the top face. Cable routing, the actual mounting holes and keep-out for the Pi/Jetson are unknown.

## Depth camera: feasibility only (not simulated)

Also feasible within the payload. A front-facing RGB-D camera on the front of the body (top-front edge, tilted slightly down) sees low obstacles and drop-offs that a single horizontal lidar plane misses, but only a ~60–90° cone, so the robot must turn to look around, and its minimum range (a few tens of cm) matters at 0.25 m/s. For this project the lidar is the better first sensor (360° coverage, simple 2-D mapping); the camera is the natural second sensor for low obstacles. Both would use the same `Navigator` – only the "scan" producer changes (e.g. a depth image sliced into a pseudo-scan). The brochure does not name the camera model; the numbers in this paragraph are generic, not xTerra's.

## Troubleshooting

| symptom | likely cause / fix |
|---|---|
| `nav:=sensed` ignores obstacles, walks straight into them | `/scan` empty: `ros2 topic hz /scan` (expect 10 Hz). Check `libgazebo_ros_ray_sensor.so` loaded (gazebo_plugins installed). If Gazebo dropped the sensor on the lumped link, add `<gazebo reference="lidar_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>` to the URDF. |
| many `path blocked` re-plans in a row | map flicker: raise `l_hit`/`occ_thresh` in `ScanMapper`, or lower `lidar_range_max` (nav.yaml) |
| `nav: no collision-free path` | obstacles + 0.42 m keep-out close a gap, or `nav_bounds` is too small for the detour (default −1.5…3.5 m) – enlarge it |
| robot stops before the box | forward guard (speed scale → 0 at 0.10 m from the body front); normal when the planned path grazes an obstacle |
| `no progress` re-plans while it is walking | Gazebo speed < 0.0125 m/s; raise the `replan_every` parameter (nav.yaml, default 4.0 s) |
| robot flips | same `FlipMonitor` + `/reset_world` handling as before; the nav restarts from an empty map after a reset |

## Hardware notes

* `/ground_truth/odom` is a **simulation** plugin. On the real robot the pose must come from the state estimator / SLAM; the node only needs an `nav_msgs/Odometry` on `odom_topic` and the same `/scan`.
* The planner takes ~5–20 ms per plan in pure Python on a PC; on a Raspberry Pi 4B expect 5–10× slower – fine at 20 Hz for 2 m courses, tight for big maps (use `grid_res: 0.1`).
* Real lidar returns, glass/black surfaces, floor reflections and odometry drift were not modelled.

## Re-running the offline tests

```bash
pip install mujoco numpy matplotlib
cd <package>/offline_tests            # i.e. .../xterra_m2/m2_metal_description/offline_tests
python3 mj_sim.py                     # stand-up check (mass 12.659 kg, tilt ~0)
python3 test_node_mj.py sensed        # real ROS node file through the rclpy stand-in
python3 mj_hard.py                    # 6 designed cases x 2 modes
python3 mj_batch.py 2                 # 32 random-layout runs, 2 processes
```
It uses `<package>/urdf/m2_metal_description.urdf` (through `xacro` if installed); set `SVAN_URDF=/path/to/file.urdf` to use another file.

---

## Appendix – draft email to xTerra (edit before sending)

**To:** info@xterrarobotics.com  **Subject:** Svan M2 (IIT Delhi) – recommended mounting of the LiDAR / depth camera, and sensor-suite details

Hello,

We are running the Svan M2 at the SIL lab, IIT Delhi (M.Tech project under Prof. Shubhendu Bhasin), and are simulating it in ROS 2 Humble + Gazebo before moving to hardware. We want to add obstacle avoidance with a lidar and/or a depth camera. Could you please tell us:

1. **Recommended mounting position(s)** for the Velodyne LiDAR and the RGB depth camera on the Svan M2 – ideally a CAD/drawing or the URDF/xacro of the sensor suite (position in `base_link`, mounting holes, any keep-out zones for the legs or the onboard computer).
2. **Which exact sensor models** you supply (the Velodyne model, the camera model), their mass, and the power/data connections available on the robot.
3. **Compute:** which module you supply for the sensor suite (Jetson-class?), and whether ROS 2 drivers are pre-installed.
4. **Localisation:** which topic/state estimator gives the robot's odometry on hardware, and the recommended way to send velocity commands (`/cmd_vel` or joint-level torque).
5. **Mass and CoM:** the URDF totals 12.659 kg, the manual says 12 kg – which one should we use, and does the CoM change with the sensor suite fitted?
6. Any limits on payload placement (CoM position, maximum height) so the trot gait stays stable.

Thank you,
Ravikumar Padala
