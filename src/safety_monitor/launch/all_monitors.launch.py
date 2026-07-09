#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    geofence_monitor = Node(
        package="safety_monitor",
        executable="geofence_monitor",
        name="geofence_monitor",
        output="screen",
        parameters=[{
            "odom_topic": "/odom_filtered",
            "status_topic": "/safety/geofence_status",
            "geofence_vertices": [-50.0, -50.0, 50.0, -50.0, 50.0, 50.0, -50.0, 50.0],
        }],
    )

    battery_monitor = Node(
        package="safety_monitor",
        executable="battery_monitor",
        name="battery_monitor",
        output="screen",
        parameters=[{
            "battery_topic": "/fmu/out/battery_status",
            "px4_namespace": "",
            "status_topic": "/safety/battery_status",
            "low_battery_threshold": 0.25,
            "critical_battery_threshold": 0.15,
            "voltage_warn": 10.5,
        }],
    )

    ekf_health_monitor = Node(
        package="safety_monitor",
        executable="ekf_health_monitor",
        name="ekf_health_monitor",
        output="screen",
        parameters=[{
            "odom_topic": "/odom_filtered",
            "status_topic": "/safety/ekf_health",
            "max_position_covariance": 0.5,
            "max_orientation_covariance": 0.1,
            "max_velocity_covariance": 1.0,
            "consecutive_failures_threshold": 5,
        }],
    )

    comms_watchdog = Node(
        package="safety_monitor",
        executable="communication_watchdog",
        name="communication_watchdog",
        output="screen",
        parameters=[{
            "heartbeat_topic": "/px4_native/status",
            "heartbeat_timeout": 1.0,
            "failsafe_timeout": 5.0,
            "watchdog_rate": 10.0,
        }],
    )

    return LaunchDescription([
        geofence_monitor,
        battery_monitor,
        ekf_health_monitor,
        comms_watchdog,
    ])
