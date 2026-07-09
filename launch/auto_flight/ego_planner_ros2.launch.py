#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _launch_ego(context, *args, **kwargs):
    if not _as_bool(LaunchConfiguration("enable_ego_planner_ros2").perform(context)):
        return []

    ego_share = get_package_share_directory("ego_planner")
    drone_id = LaunchConfiguration("drone_id").perform(context)
    odom_suffix = LaunchConfiguration("ego_odom_suffix").perform(context)
    cloud_suffix = LaunchConfiguration("ego_cloud_suffix").perform(context)
    pos_cmd_topic = f"/drone_{drone_id}_planning/pos_cmd"
    bspline_topic = f"/drone_{drone_id}_planning/bspline"

    advanced_param = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ego_share, "launch", "advanced_param.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
            "map_size_x_": LaunchConfiguration("map_size_x"),
            "map_size_y_": LaunchConfiguration("map_size_y"),
            "map_size_z_": LaunchConfiguration("map_size_z"),
            "odometry_topic": odom_suffix,
            "cloud_topic": cloud_suffix,
            "camera_pose_topic": "unused_camera_pose",
            "depth_topic": "unused_depth",
            "max_vel": LaunchConfiguration("max_vel"),
            "max_acc": LaunchConfiguration("max_acc"),
            "planning_horizon": LaunchConfiguration("planning_horizon"),
            "flight_type": "1",
            "point_num": "1",
            "point0_x": "0.0",
            "point0_y": "0.0",
            "point0_z": "1.0",
            "use_distinctive_trajs": "True",
        }.items(),
    )

    traj_server = Node(
        package="ego_planner",
        executable="traj_server",
        name=f"drone_{drone_id}_traj_server",
        output="screen",
        remappings=[
            ("planning/bspline", bspline_topic),
            ("/position_cmd", pos_cmd_topic),
            ("position_cmd", pos_cmd_topic),
        ],
        parameters=[{"traj_server/time_forward": 1.0}],
    )

    adapter = Node(
        package="px4_offboard_bridge",
        executable="ego_planner_ros2_adapter",
        name="ego_planner_ros2_adapter",
        output="screen",
        parameters=[{
            "odom_in_topic": LaunchConfiguration("odom_topic"),
            "cloud_in_topic": LaunchConfiguration("cloud_topic"),
            "goal_in_topic": LaunchConfiguration("goal_topic"),
            "ego_odom_out_topic": f"/drone_{drone_id}_{odom_suffix}",
            "ego_cloud_out_topic": f"/drone_{drone_id}_{cloud_suffix}",
            "ego_goal_out_topic": "/move_base_simple/goal",
            "ego_position_cmd_topic": pos_cmd_topic,
            "cmd_vel_out_topic": LaunchConfiguration("cmd_vel_out_topic"),
            "max_xy_speed": LaunchConfiguration("max_xy_speed"),
            "max_z_speed": LaunchConfiguration("max_z_speed"),
            "max_yaw_rate": LaunchConfiguration("max_yaw_rate"),
        }],
    )

    return [advanced_param, traj_server, adapter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_ego_planner_ros2",
            default_value="true",
            description="Launch EGO-Planner ROS2 branch and the PX4 MID360 adapter",
        ),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("odom_topic", default_value="/odom_filtered"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_local_obstacles"),
        DeclareLaunchArgument("goal_topic", default_value="/planning/goal_pose"),
        DeclareLaunchArgument("cmd_vel_out_topic", default_value="/cmd_vel_planned"),
        DeclareLaunchArgument("ego_odom_suffix", default_value="odom_filtered"),
        DeclareLaunchArgument("ego_cloud_suffix", default_value="cloud_local_obstacles"),
        DeclareLaunchArgument("map_size_x", default_value="80.0"),
        DeclareLaunchArgument("map_size_y", default_value="40.0"),
        DeclareLaunchArgument("map_size_z", default_value="4.0"),
        DeclareLaunchArgument("max_vel", default_value="1.2"),
        DeclareLaunchArgument("max_acc", default_value="2.0"),
        DeclareLaunchArgument("planning_horizon", default_value="7.5"),
        DeclareLaunchArgument("max_xy_speed", default_value="1.2"),
        DeclareLaunchArgument("max_z_speed", default_value="0.5"),
        DeclareLaunchArgument("max_yaw_rate", default_value="0.8"),
        OpaqueFunction(function=_launch_ego),
    ])
