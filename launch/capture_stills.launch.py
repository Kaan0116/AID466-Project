"""
Fotoğraf çekme launch: simülasyonu başlatır + stills scriptini çalıştırır.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg       = get_package_share_directory('following_study')
    pkg_src   = os.path.join(os.path.dirname(pkg), '..', '..', 'src', 'following_study')
    scripts_dir = os.path.join(pkg, 'scripts')
    world_file  = os.path.join(pkg, 'worlds', 'corridor.world')
    gazebo_ros  = get_package_share_directory('gazebo_ros')

    model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    # stills scripti Gazebo başladıktan 5 s sonra başlat
    capture = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    os.path.join(scripts_dir, 'capture_stills.py'),
                    '--pkg-share', pkg,
                    '--urdf', os.path.join(pkg, 'urdf', 'robot.urdf'),
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([model_path, gazebo, capture])
