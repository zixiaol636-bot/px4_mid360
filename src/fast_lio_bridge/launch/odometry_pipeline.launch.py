#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic_in",
        default_value="/Odometry",
        description="Input FAST-LIO odometry topic",
    )
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic_in",
        default_value="/cloud_registered",
        description="Input FAST-LIO point cloud topic",
    )
    filter_alpha_arg = DeclareLaunchArgument(
        "filter_alpha",
        default_value="0.3",
        description="Low-pass filter alpha for odometry smoothing",
    )

    odometry_relay_node = Node(
        package="fast_lio_bridge",
        executable="odometry_relay",
        name="odometry_relay",
        output="screen",
        parameters=[{
            "odom_topic_in": LaunchConfiguration("odom_topic_in"),
            "odom_topic_out": "/odom_filtered",
            "odom_frame_id": "odom",
            "base_link_frame_id": "base_link",
            "filter_alpha": LaunchConfiguration("filter_alpha"),
            "publish_tf": True,
        }],
    )

    pointcloud_relay_node = Node(
        package="fast_lio_bridge",
        executable="pointcloud_relay",
        name="pointcloud_relay",
        output="screen",
        parameters=[{
            "cloud_topic_in": LaunchConfiguration("cloud_topic_in"),
            "cloud_topic_out": "/cloud_registered_relay",
            "output_frame_id": "map",
            "downsample_factor": 1,
        }],
    )

    return LaunchDescription([
        odom_topic_arg,
        cloud_topic_arg,
        filter_alpha_arg,
        odometry_relay_node,
        pointcloud_relay_node,
    ])
