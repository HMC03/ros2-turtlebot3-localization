#!/usr/bin/env python3
# TurtleBot3 simulation launch file for Gazebo with RViz.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to package directories and RViz config.
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_config_file = os.path.join(get_package_share_directory('tb3_localization'), 'rviz', 'tb3_map.rviz')

    # Sim time and robot spawn position parameters.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')

    # Gazebo world file path.
    world = os.path.join(get_package_share_directory('tb3_localization'), 'worlds', 'turtlebot3_house.world')

    # Start Gazebo server (physics) with world file.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Start Gazebo client (3D GUI).
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # Publish TurtleBot3 TF and joint states.
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3 at (x, y) in Gazebo.
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Add TurtleBot3 model path for Gazebo.
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    )

    # Bridge SetEntityPose service from Gazebo to ROS
    gz_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_service_bridge',
        output='screen',
        arguments=[
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean'
        ],
    )

    # Map server for localization
    map_file = os.path.join(get_package_share_directory('tb3_localization'), 'maps', 'map.yaml')
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )

    # Lifecycle manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server']}]
    )

    # Launch RViz with TurtleBot3 config.
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Build and return launch description.
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gz_service_bridge)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)
    return ld