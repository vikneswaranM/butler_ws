#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to the slam_toolbox launch file
    executable_ = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # Path to the parameters file inside this package
    params_file = os.path.join(
        get_package_share_directory('butler_bot'),
        'config',
        'mapping_params.yaml'
    )

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(executable_),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file,
        }.items(),
    )

    return LaunchDescription([slam_include])

