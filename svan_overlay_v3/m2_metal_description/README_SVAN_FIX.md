# Svan M2 – stand + trot + waypoint overlay for `xterra_m2_description`

Drop these files over your existing package folder (`.../xterra_m2/m2_metal_description`).
`urdf/` and `meshes/` are **not touched** – the exact xTerra URDF and STL meshes are still used.

## What was wrong (diagnosis)
1. **Derivative gain too high for ROS latency.** Your PD law (`kd = 2 / 2 / 2.5`) is stable only if the torque command
   reaches the joint in the same 2 ms tick. In Gazebo it always arrives 1-3 ticks late (joint_states -> Python node ->
   topic -> controller_manager). On a MuJoCo model built from your URDF the original law holds 1.9 deg pitch at 0 ticks
   of delay, but with 1-3 ticks it chatters at +-22 Nm (calf saturated), the body drops to 0.13-0.17 m and tilts 12-23 deg.
2. **Start-up gap.** The node started ~4 s (wall) after spawn. Until then the joints are held at q=0 (straight legs)
   by gazebo_ros2_control, then limp. The robot fell from straight legs and the PD had to recover from a sprawl.
3. **Mesh collisions** on all 17 links: slow ODE (real-time factor 0.2) and jittery foot contact.
4. Wall-clock timer + `time.time()` ramp: node time and sim time ran at different speeds (RTF 0.2).
5. Stand pose had the feet under the hips while the CoM is 5.6 cm forward -> front legs carried ~62 % of the weight.

## What changed
| file | change |
|---|---|
| `scripts/compact_urdf.py` | simulation overlay applied while generating the URDF: primitive collisions (1 box body + 4 foot spheres), joint damping 1.0, stand pose via ros2_control `initial_value` (or `--zero-offset`), foot friction/contact, ground-truth odometry plugin. Flags: `--mesh-collisions`, `--no-initial-pose`, `--zero-offset`, `--manual-limits`, `--damping`. |
| `scripts/svan_core.py` | ROS-free controller: exact analytic IK, balanced stand, gravity feed-forward, trot (diagonal pairs), waypoint follower. The same file is what I tested in MuJoCo. |
| `scripts/svan_walk_controller.py` | ROS node (replaces `svan_pid_controller.py`, which is still installed but no longer launched). Event-driven on `/joint_states`, uses sim time. |
| `launch/gazebo.launch.py` | custom world, `use_sim_time`, node started together with the controller spawners, args `mode`, `spawn_z`, `initial_pose`, `gui`. |
| `worlds/svan_flat.world` | flat ground, ODE 1 ms step, friction 1.0 |
| `config/walk.yaml` | waypoints, speeds, gains |
| `CMakeLists.txt`, `package.xml` | install new files, add deps |

## Apply and run (WSL, ROS 2 Humble)
```bash
# 1. find your package folder and copy the overlay into it
PKG=$(find ~ -type d -name m2_metal_description -path "*xterra_m2*" | head -1); echo "$PKG"
unzip -o ~/svan_overlay.zip -d /tmp/svan_overlay
cp -r /tmp/svan_overlay/m2_metal_description/. "$PKG"/

# 2. build from the SAME workspace root you built before (the folder that contains build/ install/ src/)
cd ~/ros2_ws          # <- adjust
colcon build --symlink-install --packages-select xterra_m2_description
source install/setup.bash

# 3. run
ros2 launch xterra_m2_description gazebo.launch.py                 # stand 6 s, then follow the square
ros2 launch xterra_m2_description gazebo.launch.py mode:=stand     # only stand
ros2 launch xterra_m2_description gazebo.launch.py gui:=false      # headless = faster
```
Drive it by hand (works in any mode, overrides the path while keys are pressed):
```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard      # i = forward, j/l = turn, k = stop
```

## What you should see in the terminal (checklist)
* (default `initial_pose:=offset`) no `found initial value` lines - the URDF itself is in the stand pose, `/joint_states` reads about 0 when standing. With `initial_pose:=true` the 12 `found initial value` lines appear, but in the 15 Oct logs Gazebo did NOT apply them (robot started with straight legs).
* `first /joint_states received: ramping to the stand pose` (+ an INFO that the legs sagged while the controller started: normal)
* the 2 s status line now also shows `z=` (0.25 = standing, ~0.07 = lying) and `tilt=` (0 = upright)
* every 2 s: `t=  8.0s  path  pos=( 0.26, 0.00) yaw= -1.1deg wp=0/4 cmd=(0.25,0.04) |tau|max= 8.0Nm`
* `ros2 topic echo /ground_truth/odom --once` prints a pose. If it does not, the node prints a warning and dead-reckons (drifts).
* Expected steady stand: base z ~ 0.25 m, torques ~5 Nm on the calves. `|tau|max` pinned at 11.9 / 23.8 Nm = something is wrong.

## If something is different
| symptom | fix |
|---|---|
| robot flipped / lying on its side, legs move but it does not travel | The node now detects this (`ROBOT IS FLIPPED`), switches the torques off and calls `/reset_world` (up to 3 times), then stands up and restarts the path by itself. By hand: `ros2 service call /reset_world std_srvs/srv/Empty`. Switch off with `auto_reset: false` in `config/walk.yaml`. |
| `gzserver died exit 255`, `Entity [svan_m2] already exists`, `Controller already loaded` | A Gazebo from an earlier run is still alive. The launch file now kills leftover gzserver / gzclient / svan_walk_controller / robot_state_publisher before it starts (set `SVAN_KEEP_GAZEBO=1` to disable). By hand: `pkill -9 -x gzserver; pkill -9 -x gzclient` |
| robot spawns with straight legs / node warns that the initial pose was not applied | You launched with `initial_pose:=true` (ros2_control `initial_value`); this gazebo_ros2_control does not apply it. Use the default `initial_pose:=offset`. |
| legs buzz / `|tau|max` saturates | lower `kd` in `config/walk.yaml` by 30 %, or raise `--damping` in the launch's compact_urdf call (1.0 -> 1.5) |
| robot sinks slowly | raise `kp` by 20 % or check `ff_scale: 1.0` |
| feet slip while walking | `--foot-mu 1.2` or lower `v_max` / `step_height` |
| no odometry | check `ros2 topic list | grep ground_truth`; the plugin prints one error for the link name that does not exist (`base` or `base_link`) - one of the two works |
| still slow (RTF < 0.5) | `gui:=false`; WSL GPU drivers; close other apps |

## Tuning knobs (`config/walk.yaml`)
`v_max` forward speed (0.25 m/s tested up to 0.35), `w_max` turn rate, `gait_period` (0.5 s), `step_height` (0.05 m),
`kp`, `kd`, `waypoints` (flat list x0,y0,x1,y1,... in the world frame, robot starts at 0,0 facing +x).

## What is verified and what is not
Verified (MuJoCo, built from the processed URDF; 500 Hz zero-order-hold torques, 1 ms physics, explicit joint damping, 0-4 ticks latency):
stands at z = 0.251 m with 0.0 deg tilt; trots forward/backward/turning, up to 0.3 m/s; follows a 1 m square with 0.9-1.8 cm mean cross-track error and all
waypoints reached; the ROS node class itself was run (with a stub of rclpy) against the simulation with shuffled joint order.
**Not verified:** a real Gazebo run (no ROS/Gazebo in my environment). Items that depend on Gazebo behaviour: `initial_value`
being applied (confirmed in the gazebo_ros2_control Humble source), the p3d plugin link name, ODE contact parameters, and the real latency.
Send me the terminal output + a screen recording if anything looks off and I will adjust.

## Hardware later
`svan_core.py` only needs q, dq (and a pose estimate for the path). On the robot, replace the node's I/O with the xTerra SDK
(`SensorData` in, `tau_ff + kp*(q_ref-q) + kd*(dq_ref-dq)` out at >= 200 Hz) and use the estimator's pose instead of `/ground_truth/odom`.
The gains here are tuned for simulation; start with much lower gains on hardware.
