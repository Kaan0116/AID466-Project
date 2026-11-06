"""
Still capture (STEP 5): starts Gazebo WITHOUT a follower / person driver / spawned entities,
then runs scripts/capture_stills.py, which owns the only follower_node and spawns the
person + robot itself for every gap. Everything shuts down when the script finishes.

  ros2 launch following_study capture_stills.launch.py            # with gzclient
  ros2 launch following_study capture_stills.launch.py gui:=false # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('following_study')
    gui = LaunchConfiguration('gui')
    seed = LaunchConfiguration('seed')
    out_dir = LaunchConfiguration('out_dir')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'simulation.launch.py')),
        launch_arguments={'gui': gui, 'seed': seed, 'start_follower': 'false',
                          'start_driver': 'false', 'spawn': 'false'}.items())

    capture = Node(package='following_study', executable='capture_stills.py',
                   arguments=['--out-dir', out_dir],
                   parameters=[{'use_sim_time': True}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('out_dir',
                              default_value=os.path.expanduser('~/ros2_ws/results/stills')),
        sim,
        capture,
        RegisterEventHandler(OnProcessExit(target_action=capture,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
