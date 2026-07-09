# PX4 + MID360 Indoor Warehouse Inventory Stack

This ROS 2 Humble workspace targets indoor inventory flight with:

- PX4 flight controller
- Livox MID360 LiDAR
- onboard computer
- FAST-LIO2 odometry and mapping
- manual waypoint inventory missions
- EGO-Planner based 3D local trajectory planning integration
- native PX4 ROS 2 XRCE-DDS communication through `px4_msgs`

The legacy MAVLink ROS bridge has been removed from this project. The PX4 link is now the PX4-recommended ROS 2 path: PX4 uORB topics exposed as `/fmu/in/*` and `/fmu/out/*` through uXRCE-DDS.

## Packages

- `mid360_driver_wrapper`: Livox MID360 bringup and watchdog helpers.
- `fast_lio_bridge`: FAST-LIO odometry and point cloud bridge utilities.
- `map_manager`: PCD map saving, loading, and occupancy map conversion.
- `relocalizer`: ICP relocalization against a saved map.
- `waypoint_planner`: waypoint YAML tools, mission generation, mission execution, and A* route optimization reserve.
- `px4_offboard_bridge`: PX4 XRCE-DDS offboard bridge, visual odometry bridge, status bridge, EGO-Planner path follower, and final 3D safety filter.
- `safety_monitor`: geofence, battery, EKF, and communication watchdog nodes.
- `warehouse_utils`: shared helpers.
- `px4_mid360`: top-level launch and config package.

## Main Control Chain

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

Localization feedback:

```text
FAST-LIO2
  -> /odom_filtered
  -> px4_visual_odometry_bridge
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
```

PX4 state feedback:

```text
PX4
  -> /fmu/out/vehicle_status
  -> px4_status_bridge
  -> /px4_native/status
  -> /px4_native/armed
```

## Build

```bash
cd ~/ros2_ws/src
git clone <this-repo> px4_mid360
cd ~/ros2_ws
vcs import src < src/px4_mid360/dependencies.repos
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Typical Workflow

1. Pilot manually flies the six warehouses and records MID360/odometry data.
2. Replay the bag and build a map with FAST-LIO2.
3. Save the PCD map and optionally convert it to an occupancy grid for the A* route optimization reserve.
4. Manually set or generate inventory waypoints.
5. Start localization, EGO-Planner integration, PX4 DDS bridge, 3D safety filter, and safety monitors.
6. Pilot takes off and hands control to Offboard.
7. `mission_executor` flies the warehouse inventory route and returns home.

Offline mapping:

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

Online mapping is optional:

```bash
ros2 launch px4_mid360 online_mapping.launch.py \
  map_path:=./maps/warehouse_online_map.pcd
```

Generate the six-warehouse serpentine inventory mission:

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

Start the full autonomous stack:

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd
```

Start the full stack with the integrated ROS2 EGO-Planner branch:

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  enable_ego_planner_ros2:=true
```

When `enable_ego_planner_ros2:=true`, the project starts `ego-planner-swarm` `ros2_version`, relays FAST-LIO odometry/point cloud into EGO-Planner topics, forwards `/planning/goal_pose` to `/move_base_simple/goal`, converts EGO `PositionCommand` into `/cmd_vel_planned`, and keeps `local_3d_safety_filter` as the final stop/slowdown layer before PX4 receives `/cmd_vel_safe`.

For map saving, `map_saver` also defaults to `cloud_frame_mode: world`, so FAST-LIO registered clouds are accumulated directly instead of being transformed by odometry a second time.

Start the mission:

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

## Documentation

- `docs/setup_guide.md`: environment setup and dependency notes.
- `docs/px4_ros2_dds_guide.md`: PX4 XRCE-DDS, `px4_msgs`, QoS, and topic rules.
- `docs/ego_planner_integration.md`: EGO-Planner integration contract and flight-test order.
- `docs/code_reading_roadmap.md`: file-by-file reading order.
- `docs/hands_on_workbook.md`: command-by-command experiment workbook.
- `docs/flight_checklist.md`: pre-flight verification checklist.
- `docs/inventory_workflow.md`: six-warehouse inventory workflow and current capability status.
- `docs/learning_guide.md`: learner-friendly architecture guide.
- `docs/learner_full_guide.md`: full learner workflow guide.

## Important Notes

- `/fmu/out/*` subscriptions must use PX4-compatible sensor-data QoS. This project uses `rmw_qos_profile_sensor_data` for PX4 status and battery subscribers.
- `/fmu/in/*` publishers do not need the same subscription QoS workaround, but message definitions must match the PX4 firmware version.
- `px4_msgs` must match the PX4 firmware branch. If messages do not compile, align `px4_msgs` with the firmware.
- PX4 DDS topics must also exist in the firmware `dds_topics.yaml`; check with `ros2 topic list | grep /fmu`.
- If PX4 starts `uxrce_dds_client` with a namespace, set `px4_namespace` in the bridge parameters.
- A* route optimization exists as a technical reserve for other scenarios. The default warehouse inventory route is still generated/manual waypoint based.
- EGO-Planner ROS2 is integrated through `ego-planner-swarm` `ros2_version` and `ego_planner_ros2_adapter`. The upstream branch recommends CycloneDDS; install `ros-humble-rmw-cyclonedds-cpp` and set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` before serious tests.
- Geofence monitoring reports safety state by default. Do not enable direct control override until a command arbiter is added, otherwise it can compete with the mission/planner chain.
- Always validate ENU/FLU to NED/FRD direction conversion at low speed before real autonomous inventory flight.
