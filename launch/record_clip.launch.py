"""
Clip recording (STEP 6): one 20 s rear-camera clip for one gap, person walking.

  ros2 launch following_study record_clip.launch.py gap:=1.2
  ros2 launch following_study record_clip.launch.py gap:=0.6 gui:=false

Starts the simulation (one follower with target_gap = gap, person_driver with the clip
speed, person at the clip start x), then scripts/record_clip.py, which waits for
steady_state (or the warm-up timeout) and records exactly duration*fps frames by
simulation-time stamps. Shuts down when the recording is finished.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from following_study import study_config


def _setup(context):
    pkg = get_package_share_directory('following_study')
    clips = study_config.study(study_config.load())['clips']
    gap = LaunchConfiguration('gap').perform(context)
    out_dir = LaunchConfiguration('out_dir').perform(context)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui').perform(context),
            'seed': LaunchConfiguration('seed').perform(context),
            'target_gap': gap,
            'person_speed': str(clips['person_speed']),
            'person_x': str(clips['person_x']),
            'record_bag': LaunchConfiguration('record_bag').perform(context),
            'bag_path': os.path.join(out_dir, f'bag_gap_{float(gap):.1f}m'),
            'log_csv': os.path.join(out_dir, f'follower_gap_{float(gap):.1f}m.csv'),
        }.items())

    recorder = Node(package='following_study', executable='record_clip.py',
                    arguments=['--gap', gap, '--out-dir', out_dir],
                    parameters=[{'use_sim_time': True}], output='screen')
    return [sim, recorder,
            RegisterEventHandler(OnProcessExit(target_action=recorder,
                                               on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gap', default_value='1.2'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('out_dir',
                              default_value=os.path.expanduser('~/ros2_ws/results/videos')),
        OpaqueFunction(function=_setup),
    ])
