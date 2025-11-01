#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_pkg_dir = get_package_share_directory('gazebo_ros')


    package_name = 'butler_bot'
    pkg_share_dir = get_package_share_directory(package_name)
    world_file = os.path.join(pkg_share_dir, 'worlds', 'French_door_cafe.world')

    # Robot description from xacro
    urdf_file_name = 'robot.urdf.xacro'
    urdf_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_description_param = {'robot_description': robot_description_content}

    # Robot State Publisher (provides robot_description for spawn)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    
    # Spawn Mobilebot in Gazebo
    spawn_mobilebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'butler'],
    )

    # Launch Gazebo server with the house world
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_dir, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Launch Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pkg_dir, 'launch', 'gzclient.launch.py'))
    )



    # Create and return launch description
    ld = LaunchDescription()

    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_mobilebot)


    return ld

