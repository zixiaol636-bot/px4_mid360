#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    icp_max_corr_dist_arg = DeclareLaunchArgument(
        "icp_max_corr_dist",
        default_value="1.0",
        description="ICP max correspondence distance in meters",
    )

    icp_max_iter_arg = DeclareLaunchArgument(
        "icp_max_iter",
        default_value="50",
        description="ICP maximum iteration count",
    )

    icp_fitness_thresh_arg = DeclareLaunchArgument(
        "icp_fitness_thresh",
        default_value="0.15",
        description="Maximum acceptable ICP fitness score",
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Map frame id",
    )

    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="Odometry frame id",
    )

    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.2",
        description="Voxel size for point cloud downsampling",
    )

    correction_rate_arg = DeclareLaunchArgument(
        "correction_rate",
        default_value="0.5",
        description="Continuous correction rate in Hz after initial alignment",
    )

    icp_relocalizer_node = Node(
        package="relocalizer",
        executable="icp_relocalizer",
        name="icp_relocalizer",
        output="screen",
        parameters=[{
            "icp_max_corr_dist": ParameterValue(
                LaunchConfiguration("icp_max_corr_dist"), value_type=float),
            "icp_max_iter": ParameterValue(
                LaunchConfiguration("icp_max_iter"), value_type=int),
            "icp_fitness_thresh": ParameterValue(
                LaunchConfiguration("icp_fitness_thresh"), value_type=float),
            "map_frame": LaunchConfiguration("map_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "voxel_leaf_size": ParameterValue(
                LaunchConfiguration("voxel_leaf_size"), value_type=float),
            "correction_rate": ParameterValue(
                LaunchConfiguration("correction_rate"), value_type=float),
        }],
    )

    return LaunchDescription([
        icp_max_corr_dist_arg,
        icp_max_iter_arg,
        icp_fitness_thresh_arg,
        map_frame_arg,
        odom_frame_arg,
        voxel_leaf_size_arg,
        correction_rate_arg,
        icp_relocalizer_node,
    ])
