#!/usr/bin/env python3
# TurtleBot3 simulation launch file for Gazebo with RViz and localization support.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    # Paths to package directories and configs.
    pkg_tb3_localization = FindPackageShare('tb3_localization')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    launch_file_dir = os.path.join(pkg_turtlebot3_gazebo, 'launch')
    rviz_config_file = PathJoinSubstitution([pkg_tb3_localization, 'rviz', 'tb3_localization.rviz'])
    map_file = PathJoinSubstitution([pkg_tb3_localization, 'maps', 'map.yaml'])

    # Sim time and robot spawn position parameters.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')

    # Gazebo world file path.
    world = PathJoinSubstitution([pkg_tb3_localization, 'worlds', 'turtlebot3_house.world'])

    # Start Gazebo server (physics) with world file.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Start Gazebo client (3D GUI).
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
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
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )

    # Bridge SetEntityPose service from Gazebo to ROS.
    gz_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_service_bridge',
        output='screen',
        arguments=[
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean'
        ],
    )

    # # Map server to publish the static map on /map.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
            'topic_names.map': '/map',
            'frame_id': 'map',
            'use_timestamp_from_file': False,
        }],
        emulate_tty=True,
    )

    configure_map = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
            output='screen'
        )]
    )

    activate_map = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
            output='screen'
        )]
    )

    # static transform
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch RViz with TurtleBot3 config (ensure it has Map display on /map and PoseArray on /particle_cloud).
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
    ld.add_action(map_server)
    ld.add_action(configure_map)
    ld.add_action(activate_map)
    ld.add_action(static_transform_publisher)
    ld.add_action(rviz_cmd)
    return ld