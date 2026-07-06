#!/usr/bin/env python3
"""
建图阶段 — ros2 bag 录制启动文件
在人工飞行时录制建图所需的全部话题。
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_name = DeclareLaunchArgument(
        'bag_name',
        default_value='warehouse_mapping',
        description='Output bag name prefix')

    output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='./bags/',
        description='Output directory for ros2 bag')

    record_topics = [
        '/livox/lidar',
        '/livox/imu',
        '/Odometry',
        '/cloud_registered',
        '/tf',
        '/tf_static',
        '/mavros/imu/data',
        '/mavros/local_position/pose',
    ]

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_name'),
             '-s', 'mcap'] + record_topics,
        cwd=LaunchConfiguration('output_dir'),
        output='screen')

    return LaunchDescription([
        bag_name,
        output_dir,
        bag_record,
    ])
