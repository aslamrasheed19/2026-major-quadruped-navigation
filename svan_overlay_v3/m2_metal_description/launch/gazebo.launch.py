import os
import subprocess
import sys
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def kill_stale_gazebo():
    """A gzserver / gzclient / walk-controller left over from an earlier run (Ctrl+C or a closed window does not always
    stop gzserver) makes the next launch fail ("Entity [svan_m2] already exists", "Controller already loaded") or start
    with a robot that is already lying on its side.  This runs once, before anything of THIS launch has started.
    Set SVAN_KEEP_GAZEBO=1 to switch it off (e.g. if you run a second, unrelated Gazebo)."""
    if os.environ.get('SVAN_KEEP_GAZEBO'):
        return
    killed = False
    for args in (['pkill', '-9', '-x', 'gzserver'], ['pkill', '-9', '-x', 'gzclient'],
                 ['pkill', '-9', '-f', 'svan_walk_controller'], ['pkill', '-9', '-f', 'robot_state_publisher']):
        try:
            killed |= subprocess.run(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
        except OSError:
            return
    if killed:
        print('[gazebo.launch.py] stopped a leftover Gazebo / Svan process from an earlier run')
        time.sleep(2.0)


def _triples(flat, n):
    flat = [float(v) for v in (flat or [])]
    if len(flat) % n:
        raise RuntimeError('nav_config: a list has %d numbers, which is not a multiple of %d' % (len(flat), n))
    return [tuple(flat[i:i + n]) for i in range(0, len(flat), n)]


def launch_setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)
    nav, initial_pose, mode = lc('nav'), lc('initial_pose'), lc('mode')
    if nav not in ('none', 'known', 'sensed'):
        raise RuntimeError("nav must be none, known or sensed (got '%s')" % nav)
    if lc('scenario') not in ('none', 'stand', 'trot'):
        raise RuntimeError("scenario must be none, stand or trot (got '%s')" % lc('scenario'))
    if lc('scenario') != 'none' and nav != 'none':
        raise RuntimeError('scenario:=%s runs a fixed open-loop command; use it with nav:=none' % lc('scenario'))
    lidar_arg = lc('lidar').lower()
    lidar = (nav == 'sensed') if lidar_arg == 'auto' else lidar_arg in ('true', '1', 'yes')
    if nav == 'sensed' and not lidar:
        raise RuntimeError('nav:=sensed needs the lidar (lidar:=auto or lidar:=true)')

    pkg = get_package_share_directory('xterra_m2_description')
    pkg_prefix = get_package_prefix('xterra_m2_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    libdir = os.path.join(pkg_prefix, 'lib', 'xterra_m2_description')
    sys.path.insert(0, libdir)

    urdf_path = os.path.join(pkg, 'urdf', 'm2_metal_description.urdf')
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    compact_urdf = os.path.join(libdir, 'compact_urdf.py')
    walk_yaml = os.path.join(pkg, 'config', 'walk.yaml')
    nav_yaml = lc('nav_config') or os.path.join(pkg, 'config', 'nav.yaml')

    # ---- URDF: --zero-offset / --no-initial-pose as before, plus --lidar ------------------------------------------
    flags = '--zero-offset' if initial_pose == 'offset' else ('' if initial_pose == 'true' else '--no-initial-pose')
    if lidar:
        flags += ' --lidar'
    robot_description = ParameterValue(Command(['python3 ', compact_urdf, ' ', urdf_path, ' ', flags]), value_type=str)

    # ---- world: flat ground, plus the obstacles of nav.yaml when navigation is on ---------------------------------
    world_path = os.path.join(pkg, 'worlds', 'svan_flat.world')
    actions = []
    if nav != 'none':
        if not os.path.exists(nav_yaml):
            raise RuntimeError('nav_config file not found: %s' % nav_yaml)
        p = yaml.safe_load(open(nav_yaml))['svan_walk_controller']['ros__parameters']
        circles = _triples(p.get('obstacle_circles'), 3)
        boxes = _triples(p.get('obstacle_boxes'), 4)
        import svan_nav
        text = open(world_path).read()
        assert '</world>' in text, 'svan_flat.world has no </world>'
        text = text.replace('</world>', svan_nav.obstacle_models_sdf(circles, boxes) + '\n  </world>')
        world_path = os.path.join(tempfile.gettempdir(), 'svan_world_nav.world')
        with open(world_path, 'w') as f:
            f.write(text)
        actions.append(LogInfo(msg='[gazebo.launch.py] nav:=%s  lidar=%s  %d cylinder(s) + %d box(es) from %s -> %s' % (
            nav, lidar, len(circles), len(boxes), nav_yaml, world_path)))
    else:
        actions.append(LogInfo(msg='[gazebo.launch.py] nav:=none  lidar=%s' % lidar))

    gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=os.pathsep.join(['/usr/share/gazebo-11/models', os.path.dirname(pkg)]))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items())
    gazebo_client = ExecuteProcess(cmd=['gzclient'], output='screen', condition=IfCondition(LaunchConfiguration('gui')))

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'svan_m2', '-z', LaunchConfiguration('spawn_z')])

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '10'])

    effort_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['effort_controller', '--controller-manager', '/controller_manager', '--param-file', controllers_yaml,
                   '--controller-manager-timeout', '10'])

    # stand + trot + waypoint following (+ obstacle avoidance when nav != none); see scripts/svan_core.py, svan_nav.py
    params = [walk_yaml]
    if nav != 'none':
        params.append(nav_yaml)                                   # replaces waypoints, adds the obstacle lists
    params.append({
        'mode': mode,
        'use_sim_time': True,
        'zero_offset': initial_pose == 'offset',
        'nav_mode': nav,
        'payload_kg': 0.83 if lidar else 0.0,                     # the lidar's mass -> gravity feed-forward scale
        'scenario': lc('scenario'),                               # none | stand | trot  (comparison runs, see COMPARISON_PROTOCOL.md)
        'log_csv': lc('log_csv'),
    })
    walk_controller = Node(package='xterra_m2_description', executable='svan_walk_controller.py', output='screen', parameters=params)

    start_controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_entity,
        on_exit=[joint_state_broadcaster, effort_controller,
                 # start the node together with the spawners: it only needs /joint_states, and its first
                 # command is already waiting when the effort controller activates (no limp gap)
                 walk_controller]))

    return actions + [gazebo_model_path, gazebo, gazebo_client, robot_state_publisher, spawn_entity, start_controllers]


def generate_launch_description():
    kill_stale_gazebo()
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless.'),
        DeclareLaunchArgument(
            'mode', default_value='path',
            description='"path" = stand, then follow the waypoints; "stand" = just stand (you can still drive it with /cmd_vel in either mode)'),
        DeclareLaunchArgument(
            'initial_pose', default_value='offset',
            description='offset (default) = the URDF joint zero IS the stand pose, so the robot is born standing '
                        '(needs no gazebo_ros2_control feature); '
                        'true = ros2_control initial_value (your log shows this is NOT applied -> robot starts with '
                        'straight legs inside the floor and can be thrown on its back); '
                        'false = original straight-leg zero pose'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.258',
            description='Spawn height of base_link [m]. 0.258 = feet 5 mm above the ground in the stand pose.'),
        DeclareLaunchArgument(
            'nav', default_value='none',
            description='none = follow the waypoints, ignore obstacles (old behaviour); '
                        'known = obstacles from config/nav.yaml are given to the planner; '
                        'sensed = same obstacles are in the world but the robot only sees them with the lidar (/scan)'),
        DeclareLaunchArgument(
            'lidar', default_value='auto',
            description='auto = lidar only for nav:=sensed; true = always add the simulated 2-D lidar; false = never'),
        DeclareLaunchArgument(
            'nav_config', default_value='',
            description='path to your own nav.yaml (same layout as config/nav.yaml); empty = the installed one'),
        DeclareLaunchArgument(
            'scenario', default_value='none',
            description='none = normal operation; stand = 6 s stand, trot = 12 s open-loop straight trot at 0.20 m/s from t=4 s. '
                        'Either one writes a svan_log v1 CSV for the Gazebo / MATLAB / Simscape comparison (use nav:=none)'),
        DeclareLaunchArgument(
            'log_csv', default_value='',
            description='CSV path for scenario runs; empty = ~/svan_logs/svan_<scenario>_gazebo.csv'),
        OpaqueFunction(function=launch_setup),
    ])
