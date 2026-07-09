#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.12",
        description="Obstacle cloud voxel leaf size; 0 disables downsampling",
    )
    obstacle_cloud_out_arg = DeclareLaunchArgument(
        "obstacle_cloud_topic_out",
        default_value="/cloud_local_obstacles",
        description="Filtered local obstacle cloud for EGO-Planner and safety filter",
    )
    crop_min_x_arg = DeclareLaunchArgument("crop_min_x", default_value="-8.0")
    crop_max_x_arg = DeclareLaunchArgument("crop_max_x", default_value="8.0")
    crop_min_y_arg = DeclareLaunchArgument("crop_min_y", default_value="-8.0")
    crop_max_y_arg = DeclareLaunchArgument("crop_max_y", default_value="8.0")
    crop_min_z_arg = DeclareLaunchArgument("crop_min_z", default_value="-2.0")
    crop_max_z_arg = DeclareLaunchArgument("crop_max_z", default_value="2.0")
    outlier_filter_type_arg = DeclareLaunchArgument(
        "outlier_filter_type",
        default_value="none",
        description="Obstacle cloud outlier filter: none, radius, or statistical",
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
            "filter_alpha": ParameterValue(
                LaunchConfiguration("filter_alpha"), value_type=float),
            "publish_tf": True,
        }],
    )

    cloud_preprocessor_node = Node(
        package="fast_lio_bridge",
        executable="cloud_preprocessor",
        name="cloud_preprocessor",
        output="screen",
        parameters=[{
            "cloud_topic_in": LaunchConfiguration("cloud_topic_in"),
            "cloud_topic_out": LaunchConfiguration("obstacle_cloud_topic_out"),
            "odom_topic": "/odom_filtered",
            "pointcloud_frame_mode": "world",
            "output_frame_id": "",
            "enable_local_crop": True,
            "crop_min_x": ParameterValue(LaunchConfiguration("crop_min_x"), value_type=float),
            "crop_max_x": ParameterValue(LaunchConfiguration("crop_max_x"), value_type=float),
            "crop_min_y": ParameterValue(LaunchConfiguration("crop_min_y"), value_type=float),
            "crop_max_y": ParameterValue(LaunchConfiguration("crop_max_y"), value_type=float),
            "crop_min_z": ParameterValue(LaunchConfiguration("crop_min_z"), value_type=float),
            "crop_max_z": ParameterValue(LaunchConfiguration("crop_max_z"), value_type=float),
            "enable_voxel_filter": True,
            "voxel_leaf_size": ParameterValue(
                LaunchConfiguration("voxel_leaf_size"), value_type=float),
            "outlier_filter_type": LaunchConfiguration("outlier_filter_type"),
        }],
    )

    return LaunchDescription([
        odom_topic_arg,
        cloud_topic_arg,
        filter_alpha_arg,
        voxel_leaf_size_arg,
        obstacle_cloud_out_arg,
        crop_min_x_arg,
        crop_max_x_arg,
        crop_min_y_arg,
        crop_max_y_arg,
        crop_min_z_arg,
        crop_max_z_arg,
        outlier_filter_type_arg,
        odometry_relay_node,
        cloud_preprocessor_node,
    ])
