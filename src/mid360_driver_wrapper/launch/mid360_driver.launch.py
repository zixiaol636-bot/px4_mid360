#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mid360_driver_wrapper")
    default_config_path = os.path.join(pkg_dir, "config", "MID360_config.json")

    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Absolute path to MID360_config.json",
    )

    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="/livox/lidar",
        description="Expected point cloud topic published by livox_ros_driver2",
    )

    livox_node = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_ros_driver2",
        output="screen",
        parameters=[{
            "xfer_format": 0,
            "multi_topic": 0,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
        }],
        arguments=[
            "--ros-args",
            "--params-file",
            LaunchConfiguration("config_path"),
        ],
    )

    driver_monitor_node = Node(
        package="mid360_driver_wrapper",
        executable="driver_monitor",
        name="driver_monitor",
        output="screen",
        parameters=[{
            "lidar_topic": LaunchConfiguration("lidar_topic"),
            "watchdog_timeout": 2.0,
            "check_rate": 10.0,
            "auto_restart": True,
            "driver_launch_command": "",
            "publish_health": True,
        }],
    )

    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=livox_node,
            on_exit=[
                LogInfo(msg="livox_ros_driver2 exited, shutting down launch."),
                EmitEvent(event=Shutdown()),
            ],
        )
    )

    return LaunchDescription([
        config_arg,
        lidar_topic_arg,
        livox_node,
        driver_monitor_node,
        exit_handler,
    ])
