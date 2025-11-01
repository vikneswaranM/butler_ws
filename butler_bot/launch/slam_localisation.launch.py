#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

     # Paths
    package_name = 'butler_bot'
    pkg_share_dir = get_package_share_directory(package_name)
    urdf_file_name = 'robot.urdf.xacro'
    urdf_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)
    
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_description_param = {'robot_description': robot_description_content}

    rviz_config_file = os.path.join(pkg_share_dir, 'config', 'map.rviz')

    declare_map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(pkg_share_dir, 'maps', 'turtle_map.yaml'),
        description='Full path to the map yaml file'
    )

	# Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
	
    # Static map server (lifecycle node)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time},
        ],
    )

    # Lifecycle manager to auto-start map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # Path to the SLAM Toolbox localization launch file
    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    # Path to your params file (inside your own package)
    params_file = os.path.join(
        get_package_share_directory('butler_bot'),
        'config',
        'localisation_params.yaml'
    )

    # Include the localization launch description
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file
        }.items(),
    )

   
    ld = LaunchDescription()

    
    ld.add_action(declare_map_yaml_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)
    ld.add_action(slam_localization)    

    return ld