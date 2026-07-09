#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="/tmp",
        description="Directory used when map_file is not provided",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="",
        description="Optional fixed PCD output file path",
    )

    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/cloud_registered",
        description="Registered point cloud topic to accumulate",
    )

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/Odometry",
        description="Odometry topic used to transform incoming point clouds",
    )

    cloud_frame_mode_arg = DeclareLaunchArgument(
        "cloud_frame_mode",
        default_value="world",
        description="Use 'world' for FAST-LIO registered clouds, or 'body' for body-frame clouds",
    )

    map_saver_node = Node(
        package="map_manager",
        executable="map_saver",
        name="map_saver",
        output="screen",
        parameters=[{
            "save_directory": LaunchConfiguration("save_directory"),
            "map_file": LaunchConfiguration("map_file"),
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "cloud_frame_mode": LaunchConfiguration("cloud_frame_mode"),
        }],
    )

    return LaunchDescription([
        save_directory_arg,
        map_file_arg,
        cloud_topic_arg,
        odom_topic_arg,
        cloud_frame_mode_arg,
        map_saver_node,
    ])
