# CLAUDE.md — PX4 + MID360 Indoor Warehouse Drone

This file provides guidance to Claude Code when working in this repository.

## Architecture Overview

This is a **ROS2 Humble** (Ubuntu 22.04) project for autonomous indoor warehouse inventory flight using:
- **PX4** flight controller (Pixhawk) — attitude/position control + EKF2 sensor fusion
- **Livox MID360** — 360° FOV LiDAR (10Hz point cloud + 200Hz IMU)
- **Onboard computer** (Jetson Orin NX / x86 NUC) — all ROS2 computation

## Key Technical Decisions

- **ROS2 Humble** over ROS1 Noetic (EOL May 2025)
- **FAST-LIO2** ([Ericsii/FAST_LIO_ROS2](https://github.com/Caltech-AMBER/FAST_LIO_ROS2)) for LiDAR-inertial SLAM
- **Nav2 DWB** for local obstacle avoidance (2D costmap + Z-axis supplement)
- **MAVROS2** for PX4 bridge (vision_pose + setpoint_raw)
- **ICP scan-to-map** for relocalization against pre-built PCD map
- **FLU ↔ NED** coordinate conversion in `warehouse_utils::FrameConversions`

## Project Structure

```
px4_mid360/
├── config/          # All YAML/JSON configuration files
│   ├── fast_lio/    # FAST-LIO2 mapping + localization configs
│   ├── nav2/        # Nav2 params + costmap configs
│   ├── px4/         # PX4 EKF2 params + offboard params
│   ├── livox/       # MID360 Ethernet config
│   ├── mavros2/     # MAVROS2 plugin config
│   └── waypoints/   # Waypoint YAML files
├── launch/          # ROS2 launch files (.launch.py)
│   ├── mapping/     # record_bag, build_map_offline, save_map
│   └── auto_flight/ # bringup_all, offboard_mission, localize_only
├── src/             # ROS2 packages (7-8 packages)
│   ├── warehouse_utils/      # Shared: frame_conversions, yaml_utils
│   ├── mid360_driver_wrapper/# Livox driver lifecycle wrapper
│   ├── fast_lio_bridge/      # FAST-LIO2 output relay (TF + odom)
│   ├── map_manager/          # PCD save/load + PCD→PGM conversion
│   ├── relocalizer/          # ICP/NDT relocalization
│   ├── waypoint_planner/     # Waypoint I/O + mission state machine
│   ├── px4_offboard_bridge/  # vision_pose, offboard_ctrl, cmdvel→setpoint, z_monitor
│   └── safety_monitor/       # geofence, battery, EKF health, comm watchdog
└── scripts/         # install, health check, bag processing
```

## Build System

- **Build tool**: colcon with ament_cmake
- **Key dependencies**: Livox SDK2 (system), FAST-LIO2, Nav2, MAVROS2, PCL, yaml-cpp
- **Install**: `./scripts/install_dependencies.sh`

## Coordinate Frames

```
map (global, origin=mapping takeoff)
  ← relocalizer publishes map→odom
odom (drift-prone)
  ← FAST-LIO2 publishes odom→base_link
base_link (drone body, FLU: x=forward, y=left, z=up)
  ← static transform (LiDAR mount offset)
lidar_frame
```

**FLU → NED conversion** (for MAVROS2):
```
pos_NED = (y_FLU, x_FLU, -z_FLU)
yaw_NED = atan2(forward_x_FLU, forward_y_FLU)
```

## Workflow Summary

### Mapping
1. Bringup: livox_driver2 + mavros2 + FAST-LIO2(mapping) + bridge + ros2 bag
2. Manual flight covering all aisles with loop closures
3. Save PCD → downsample → convert to PGM/YAML for Nav2

### Autonomous Flight
1. Bringup: All modules + Nav2 (7-8 terminals)
2. Relocalize: provide initial pose → trigger ICP → verify map→odom
3. Arm → OFFBOARD → takeoff → Nav2 waypoint following (with DWB avoidance) → RTL

## PX4 Critical Parameters

See `config/px4/ekf2_params.txt`:
- `EKF2_AID_MASK=280` (vision position+yaw+velocity)
- `EKF2_MAG_TYPE=5` (disable magnetometer — indoor!)
- `COM_OBL_ACT=2` (land on offboard loss)

## Nav2 UAV Adaptation

- `use_dwa: false` — UAV is holonomic (no differential-drive constraints)
- Z-axis handled independently (Nav2 only manages X-Y)
- `cmdvel_to_setpoint` node converts Nav2 Twist → MAVROS2 PositionTarget
- `z_axis_monitor` supplements 2D DWB with hanging obstacle detection

## File Naming Conventions

- ROS2 packages: `package.xml` (format 3), `CMakeLists.txt` (ament_cmake)
- Launch files: `*.launch.py` (Python format)
- Config files: `*.yaml` (ROS2 parameter format with `ros__parameters`)
- C++ nodes: standard ROS2 rclcpp patterns with `declare_parameters`
