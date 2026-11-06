"""
Main simulation launch file. All defaults come from config/study_params.yaml.

  ros2 launch following_study simulation.launch.py
  ros2 launch following_study simulation.launch.py target_gap:=0.6 person_speed:=1.0 gui:=false

Arguments (empty = value from the YAML):
  target_gap, person_speed, person_x   study set-up
  start_follower   true/false   start follower_node (false when a script owns the follower)
  start_driver     true/false   start person_driver
  spawn            true/false   spawn person + robot (false when a script spawns them)
  gui              true/false   false = headless (gzserver only, no gzclient)
  seed             Gazebo random seed (int)
  robot_dx, robot_y, robot_yaw  extra robot start offset (evaluation jitter)
  record_bag       true/false,  bag_path   rosbag2 recording of ground truth + follower topics
  log_csv          path of the follower CSV log ('' = off)
  params_file      alternative YAML
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from following_study import study_config
from following_study.geometry import robot_spawn_x

BAG_TOPICS = ['/person/odom', '/robot/odom', '/cmd_vel', '/person/cmd_vel',
              '/follower/gap', '/follower/gap_error', '/follower/steady',
              '/person/walking', '/robot/bumper', '/scan', '/clock']

ARGS = {
    'target_gap': ('', 'Target gap [m] (robot front edge -> person back)'),
    'person_speed': ('', 'Person walking speed [m/s], 0 = static'),
    'person_x': ('', 'Person start x [m]'),
    'start_follower': ('true', 'Start follower_node'),
    'start_driver': ('true', 'Start person_driver'),
    'spawn': ('true', 'Spawn person and robot'),
    'gui': ('true', 'Start gzclient (false = headless)'),
    'seed': ('0', 'Gazebo random seed'),
    'robot_dx': ('0.0', 'Extra robot start offset along -x [m]'),
    'robot_y': ('0.0', 'Robot start y [m]'),
    'robot_yaw': ('0.0', 'Robot start yaw [rad]'),
    'record_bag': ('false', 'Record a rosbag2'),
    'bag_path': ('', 'rosbag2 output dir (default: ~/ros2_ws/results/bags/run_<time>)'),
    'log_csv': ('', 'Follower CSV log path'),
    'params_file': ('', 'Parameter YAML (default: config/study_params.yaml)'),
}


def _bool(v: str) -> bool:
    return v.strip().lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    pkg = get_package_share_directory('following_study')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}

    params_file = a['params_file'] or os.path.join(pkg, 'config', 'study_params.yaml')
    cfg = study_config.load(params_file)
    fp = study_config.follower_params(cfg)
    spawn_cfg = study_config.study(cfg)['spawn']

    target_gap = float(a['target_gap'] or fp['target_gap'])
    person_speed = float(a['person_speed'] or study_config.driver_params(cfg)['speed'])
    person_x = float(a['person_x'] or spawn_cfg['person_x'])
    robot_x = robot_spawn_x(person_x, target_gap, fp['person_back_offset'],
                            fp['robot_front_offset'],
                            spawn_cfg['spawn_offset'] + float(a['robot_dx']))

    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf')
    person_sdf = os.path.join(pkg, 'models', 'person', 'model.sdf')
    sim_time = {'use_sim_time': True}

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': os.path.join(pkg, 'worlds', 'corridor.world'),
                'seed': a['seed'],
                # lockstep: sensors render at exactly their update_rate in simulation time
                'lockstep': 'true',
                'verbose': 'false',
            }.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': open(urdf_file).read()}, sim_time],
             output='screen'),
    ]
    if _bool(a['gui']):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))))

    if _bool(a['spawn']):
        actions += [
            Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-file', person_sdf, '-entity', 'person',
                            '-x', f'{person_x}', '-y', '0.0', '-z', '0.0']),
            Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-file', urdf_file, '-entity', 'study_robot',
                            '-x', f'{robot_x}', '-y', a['robot_y'],
                            '-z', f'{spawn_cfg["robot_z"]}', '-Y', a['robot_yaw']]),
        ]

    if _bool(a['start_driver']):
        actions.append(Node(
            package='following_study', executable='person_driver', output='screen',
            parameters=[params_file, {'speed': person_speed}, sim_time]))

    if _bool(a['start_follower']):
        actions.append(Node(
            package='following_study', executable='follower_node', output='screen',
            parameters=[params_file,
                        {'target_gap': target_gap, 'v_person': person_speed,
                         'log_csv': a['log_csv']},
                        sim_time]))

    if _bool(a['record_bag']):
        bag_path = a['bag_path'] or os.path.expanduser(
            time.strftime('~/ros2_ws/results/bags/run_%Y%m%d_%H%M%S'))
        os.makedirs(os.path.dirname(bag_path), exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path] + BAG_TOPICS,
            output='screen'))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('following_study')
    decls = [DeclareLaunchArgument(k, default_value=v, description=d)
             for k, (v, d) in ARGS.items()]
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))
    return LaunchDescription(decls + [model_path, OpaqueFunction(function=_setup)])
