#!/usr/bin/env python3
"""Record mapping topics during manual warehouse flights."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_name = DeclareLaunchArgument(
        "bag_name",
        default_value="warehouse_mapping",
        description="Output bag name prefix",
    )

    output_dir = DeclareLaunchArgument(
        "output_dir",
        default_value="./bags/",
        description="Output directory for ros2 bag",
    )

    record_topics = [
        "/livox/lidar",
        "/livox/imu",
        "/Odometry",
        "/odom_filtered",
        "/cloud_registered",
        "/cloud_local_obstacles",
        "/tf",
        "/tf_static",
        "/fmu/out/vehicle_status",
        "/fmu/out/vehicle_local_position",
        "/fmu/out/vehicle_odometry",
        "/fmu/out/sensor_combined",
        "/fmu/out/battery_status",
    ]

    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            LaunchConfiguration("bag_name"),
            "-s",
            "mcap",
        ] + record_topics,
        cwd=LaunchConfiguration("output_dir"),
        output="screen",
    )

    return LaunchDescription([
        bag_name,
        output_dir,
        bag_record,
    ])
