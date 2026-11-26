# Svan M2 – common protocol for the Gazebo / MATLAB / Simscape comparison

Prof. Bhasin's request (meeting of 27 Oct 2026): finish the Simscape model and **compare its results with Gazebo and MATLAB**.
A comparison only means something if all three simulators run the *same experiment* and write the *same file*. This page fixes both.

## 1. The two scenarios

Both start with the controller's own stand-up ramp (0–3 s, from the pose the robot is born in to the stand pose) and use the **same controller**
(`svan_core.py` in Gazebo, `control/svan_ctl_step.m` in MATLAB, the same equations in Simscape): joint PD + gravity feed-forward + open-loop trot,
500 Hz, one tick (2 ms) of latency, `kp = (60, 80, 90)`, `kd = (0.3, 0.36, 0.45)`, gait period 0.5 s, duty 0.5, step height 5 cm.

| name | length | command | what it checks |
|---|---|---|---|
| `stand` (S0) | 6 s | none | the static gravity feed-forward: every simulator should hold the robot with the torques of Table 1 in Entry 3 (`FF_STAND`) |
| `trot` (S1) | 12 s | `vx = 0.20 m/s`, `vy = wz = 0` from `t = 4.0 s` to the end (open loop, no odometry feedback) | the gait: joint angles, torques and body motion in the steady trot (6–12 s) |

The trot is deliberately **open loop** (a fixed velocity command, not "follow the waypoints"): path following feeds the simulator's own position back
into the command, so small differences would grow into different paths and the three runs could no longer be compared sample by sample.

## 2. Conventions (the usual sources of "the curves do not match")

* **Time**: `t = 0` is the first control tick (start of the stand-up ramp), not the moment Gazebo was started.
* **Joint angles**: absolute, as in the URDF stand definition: stand pose `(hip, thigh, calf) = (0, 0.684, −1.738)` rad for all four legs. Gazebo's
  zero-offset URDF reports angles relative to the stand pose; the node adds the offset back before logging.
* **Joint order**: FR, FL, RR, RL, each hip, thigh, calf (`FR_hip, FR_thigh, FR_calf, FL_hip, …`). Units: rad, rad/s, N·m.
* **Torque column `tau`**: the torque *applied during the 2 ms tick that starts at `t`*, i.e. the one the controller computed one tick earlier. All three
  simulators use this convention (the MATLAB log already did, modelling the delay of `ros2_control`, where a command takes effect at the next update; the Simscape model must hold each torque for
  one controller step).
* **`qref`**: the reference angle the inverse kinematics produced at `t` (output of the gait), so tracking error is `qref − q`.
* **Base pose**: ground truth of `base_link` in the world frame, roll/pitch/yaw (Z-Y-X).
* **Foot normal force `Fn_*`**: filled where the simulator has it (MATLAB); `NaN` in Gazebo (no contact sensors in the model). Not used by the checks.

## 3. The file ("svan_log v1")

Text CSV, 10 ms per row (1201 rows for `trot`, 601 for `stand`), `#` comment lines first, then a header line, then 62 columns:

`t` · `base_x base_y base_z` · `roll pitch yaw` · `cmd_vx cmd_vy cmd_wz` · `Fn_FR Fn_FL Fn_RR Fn_RL` · `q_*` (12) · `dq_*` (12) · `qref_*` (12) · `tau_*` (12)

File names: `svan_<scenario>_<source>.csv`, e.g. `svan_trot_gazebo.csv`, `svan_trot_matlab.csv`, `svan_trot_simscape.csv`. The first comment lines say which simulator and
settings produced the file. The format is defined in `scripts/svan_log.py` (Python reader/writer used by the Gazebo node and by the compare script).

## 4. How to produce each log

**Gazebo** (after installing overlay v3 and rebuilding: `colcon build --packages-select xterra_m2_description --symlink-install`, `source install/setup.bash`):
```bash
ros2 launch xterra_m2_description gazebo.launch.py scenario:=stand gui:=false      # writes ~/svan_logs/svan_stand_gazebo.csv after 6 s of simulation time
ros2 launch xterra_m2_description gazebo.launch.py scenario:=trot  gui:=false      # writes ~/svan_logs/svan_trot_gazebo.csv after 12 s
```
The terminal prints `scenario trot finished: 1201 rows written to …`; then press Ctrl+C. Start each run from a fresh launch (the launch file already kills a leftover
Gazebo). `gui:=false` is only to save time; use `gui:=true` to watch. If the robot falls over during the run, discard that file and run again.

**MATLAB** – copy `comparison/matlab/export_log_v1.m` and `run_scenario_v1.m` into the `sim/` folder of `svan_matlab`, then:
```matlab
startup_svan
run_scenario_v1('stand');     % results/svan_stand_matlab.csv
run_scenario_v1('trot');      % results/svan_trot_matlab.csv
```
(Without MATLAB: `python3 comparison/mat_to_v1.py results/trot.mat trot out.csv` converts the stored run; scipy needed.)

**Simscape** – objective 3. The model applies the same controller torques (held for one 2 ms step) and writes the same 62 columns; the logging block is built together with the model.

## 5. Checking and comparing

```bash
python3 comparison/svan_compare.py svan_trot_gazebo.csv svan_trot_matlab.csv svan_trot_simscape.csv --plot cmp_trot.png
python3 comparison/svan_compare.py svan_stand_gazebo.csv svan_stand_matlab.csv svan_stand_simscape.csv
```
It first **checks** each file (time step, NaN, angle/torque units, zero-offset mistakes) and then prints one column per simulator: forward speed, drift, body height, roll/pitch RMS,
gait frequency, RMS and peak torque per joint type, tracking error, torque saturation, and for `stand` the deviation from `FF_STAND`. The figure overlays FR thigh/calf angle and torque and base
height over 2 s of steady trot (change with `--win 8 10`).

How to read it: the gait is open loop and the contact models differ, so **compare size, shape and period** (torque peaks, RMS, gait frequency, joint ranges, tracking error), not individual samples.
Differences in forward speed of 10–20 % between simulators are normal (foot slip); a different gait frequency, torque peaks that differ by a factor, or saturation in one simulator only point to a
real modelling difference.

## 6. What was tested (and what was not)

* The Gazebo node and logger were run **offline**: the real `svan_walk_controller.py` driven by MuJoCo physics through a stand-in for ROS (`offline_tests/test_scenario_mj.py`). All checks pass for both
  scenarios (1201 / 601 rows, 10 ms step, `tau` = previous tick's command, `cmd` schedule, static torque within 0.07 N·m of `FF_STAND`, upright, no saturation). **It has not been run in Gazebo or ROS 2 yet.**
* Pipeline check with the stored MATLAB run (`results/trot.mat`, converted) and the MuJoCo run (standing in for Gazebo):

| steady trot 6–12 s | MuJoCo (stand-in) | MATLAB |
|---|---|---|
| forward speed [m/s] | 0.157 | 0.175 |
| gait frequency [Hz] | 1.997 | 1.997 |
| torque peak hip / thigh / calf [N·m] | 4.6 / 6.4 / 12.7 | 4.7 / 6.3 / 12.5 |
| torque RMS hip / thigh / calf [N·m] | 2.61 / 3.34 / 7.16 | 2.64 / 3.38 / 7.20 |
| tracking error RMS hip / thigh / calf [rad] | 0.018 / 0.020 / 0.025 | 0.018 / 0.020 / 0.029 |
| roll / pitch RMS [rad] | 0.0082 / 0.0085 | 0.0083 / 0.0100 |

  This is *not* the Gazebo comparison; it only shows that the files, the checker and the two models work together and that the independent MATLAB equations of motion agree with MuJoCo: torques within about 2 %, gait frequency identical, forward speed and pitch within 10–20 %.
