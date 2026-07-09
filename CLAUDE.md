# Project Notes

This repository is a ROS 2 Humble workspace for indoor warehouse inventory flight using PX4, Livox MID360, FAST-LIO2, EGO-Planner integration, and native PX4 XRCE-DDS.

## Architecture

- PX4 communication: XRCE-DDS with `px4_msgs`.
- Sensor front end: Livox MID360.
- Localization and mapping: FAST-LIO2.
- Mission layer: generated or manually edited waypoint YAML.
- Local safety layer: 3D point-cloud obstacle avoidance.
- Flight execution: PX4 Offboard setpoints through `/fmu/in/*`.

## Main Topics

```text
/odom_filtered
/cloud_registered
/cmd_vel
/cmd_vel_safe
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_visual_odometry
/fmu/out/vehicle_status
/fmu/out/vehicle_command_ack
/fmu/out/battery_status
/px4_native/status
/px4_native/armed
```

## Key Dependencies

- ROS 2 Humble
- PX4 firmware with `uxrce_dds_client`
- Micro XRCE-DDS Agent
- `px4_msgs`
- `px4_ros_com`
- Livox SDK2 and `livox_ros_driver2`
- FAST-LIO2
- EGO-Planner or a ROS 2 adapter that publishes `/planning/local_path`
- PCL
- yaml-cpp

## Important Rules

- Subscribers to PX4 `/fmu/out/*` topics should use PX4-compatible sensor-data QoS.
- `px4_msgs` must match the PX4 firmware message definitions.
- Mission code publishes `/cmd_vel`; it should not publish PX4 setpoints directly.
- `ego_planner_path_follower` follows the external EGO-Planner local path, and `local_3d_safety_filter` is the final safety layer before PX4 setpoint conversion.
- Validate ENU/FLU to NED/FRD conversion before real flight.
