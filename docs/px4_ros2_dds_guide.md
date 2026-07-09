# PX4 ROS 2 XRCE-DDS Guide

This project now uses native PX4 ROS 2 communication through XRCE-DDS and `px4_msgs`. The legacy MAVLink ROS bridge has been removed.

Official references:

- PX4 uXRCE-DDS middleware: https://docs.px4.io/main/zh/middleware/uxrce_dds
- PX4 ROS 2 user guide compatibility issue: https://docs.px4.io/main/zh/ros2/user_guide#兼容性问题

## Why `px4_msgs` Is Required

PX4 does not receive generic ROS messages directly. Through XRCE-DDS, selected PX4 uORB topics are exposed as ROS 2 topics:

- `/fmu/in/*`: ROS 2 publishes commands into PX4.
- `/fmu/out/*`: PX4 publishes state back to ROS 2.

The message types come from `px4_msgs`, so the project must build with `px4_msgs`. The `px4_msgs` branch/version should match the PX4 firmware message definitions.

Current project topics:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_visual_odometry
/fmu/out/vehicle_status
/fmu/out/vehicle_command_ack
/fmu/out/battery_status
```

## QoS Rule

PX4 official documentation warns that PX4 ROS 2 publishers may be incompatible with normal ROS 2 default QoS. In practice:

- Subscribers to `/fmu/out/*` should use `rmw_qos_profile_sensor_data`.
- Publishers to `/fmu/in/*` normally do not need this subscriber QoS workaround.

This project applies the rule in:

- `src/px4_offboard_bridge/src/px4_status_bridge.cpp`
- `src/safety_monitor/src/battery_monitor.cpp`

If `ros2 topic list` shows `/fmu/out/...` but your node receives nothing, suspect QoS first.

## Namespace Rule

PX4 can prefix all DDS topics when `uxrce_dds_client` is started with a namespace, for example:

```bash
uxrce_dds_client start -n uav_1
```

Then topics become:

```text
/uav_1/fmu/in/...
/uav_1/fmu/out/...
```

The project supports this through `px4_namespace`. Leave it empty for a single aircraft. Set it to `/uav_1` when PX4 uses that DDS namespace.

## `dds_topics.yaml` Rule

`px4_msgs` only gives ROS 2 the message definitions. It does not guarantee every message is exposed by PX4. The actual topics available through XRCE-DDS are defined by the PX4 firmware `dds_topics.yaml`.

Before relying on a topic, check:

```bash
ros2 topic list | grep /fmu
```

If a topic is not listed, confirm your PX4 firmware includes it in `dds_topics.yaml`.

## Current DDS Control Chain

```text
mission_executor
  -> /planning/goal_pose
  -> ego_planner_ros2_adapter
  -> EGO-Planner ROS2
  -> /drone_0_planning/pos_cmd
  -> ego_planner_ros2_adapter
  -> /cmd_vel_planned
  -> local_3d_safety_filter
  -> /cmd_vel_safe
  -> px4_cmdvel_bridge
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
  -> PX4
```

Visual odometry chain:

```text
FAST-LIO2
  -> /odom_filtered
  -> px4_visual_odometry_bridge
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
```

Status chain:

```text
PX4
  -> /fmu/out/vehicle_status
  -> px4_status_bridge
  -> /px4_native/status
  -> /px4_native/armed
```

## Offboard Mode Notes

PX4 Offboard mode requires continuous setpoint and Offboard heartbeat publication. `px4_cmdvel_bridge` continuously publishes:

- `OffboardControlMode`
- `TrajectorySetpoint`

It also exposes services:

```bash
ros2 service call /px4_cmdvel_bridge/set_offboard std_srvs/srv/Trigger "{}"
ros2 service call /px4_cmdvel_bridge/arm std_srvs/srv/Trigger "{}"
ros2 service call /px4_cmdvel_bridge/disarm std_srvs/srv/Trigger "{}"
```

For real aircraft safety, automatic arm and automatic Offboard switching are disabled by default.

## Startup Check

Start the XRCE-DDS Agent, then check:

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_status --once
ros2 topic echo /px4_native/status --once
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

If `/fmu/out/vehicle_status` echoes but `/px4_native/status` does not, check `px4_status_bridge`.

If `/fmu/out/vehicle_status` does not echo at all, check the PX4 `uxrce_dds_client`, XRCE-DDS Agent, ROS domain, and PX4 firmware topic configuration.
