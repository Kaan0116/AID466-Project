"""
Ana simülasyon launch dosyası.
Kullanım:
  ros2 launch following_study simulation.launch.py
  ros2 launch following_study simulation.launch.py target_gap:=0.6 v_person:=1.0 headless:=true
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('following_study')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg, 'worlds', 'corridor.world')
    urdf_file  = os.path.join(pkg, 'urdf', 'robot.urdf')   # xacro → urdf build'de oluşur
    person_sdf = os.path.join(pkg, 'models', 'person', 'model.sdf')
    params_file= os.path.join(pkg, 'config', 'follower_params.yaml')

    # Launch argümanları
    target_gap_arg = DeclareLaunchArgument('target_gap', default_value='1.2')
    v_person_arg   = DeclareLaunchArgument('v_person',   default_value='0.0')
    headless_arg   = DeclareLaunchArgument('headless',   default_value='false')
    gui_arg        = DeclareLaunchArgument('gui',        default_value='true')

    target_gap = LaunchConfiguration('target_gap')
    v_person   = LaunchConfiguration('v_person')
    headless   = LaunchConfiguration('headless')

    # GAZEBO_MODEL_PATH'e model dizinini ekle
    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg, 'models') + ':' +
        os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # Gazebo başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    # Robot spawn
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_file,
            '-entity', 'study_robot',
            '-x', '1.0', '-y', '0.0', '-z', '0.225',
        ],
        output='screen',
    )

    # Kişi spawn (x=10, statik başlar)
    spawn_person = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', person_sdf,
            '-entity', 'person',
            '-x', '10.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', urdf_file])}],
        output='screen',
    )

    # Follower node
    follower = Node(
        package='following_study',
        executable='follower_node',
        parameters=[
            params_file,
            {
                'target_gap': target_gap,
                'v_person':   v_person,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        target_gap_arg,
        v_person_arg,
        headless_arg,
        gui_arg,
        model_path,
        gazebo,
        spawn_robot,
        spawn_person,
        robot_state_pub,
        follower,
    ])
