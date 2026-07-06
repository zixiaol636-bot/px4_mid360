# PX4 + Livox MID360 Warehouse Stack

ROS 2 Humble workspace for an indoor warehouse drone built around:

- PX4 flight control
- Livox MID360 LiDAR
- FAST-LIO2 odometry / mapping
- Nav2 planning
- MAVROS offboard control

The repository is organized as a multi-package ROS 2 workspace and now also
includes a top-level `px4_mid360` package so the root launch files can be used
directly with `ros2 launch px4_mid360 ...`.

## Packages

- `mid360_driver_wrapper`: Livox driver bringup and watchdog support
- `fast_lio_bridge`: odometry and point-cloud bridge utilities
- `map_manager`: map accumulation, loading, and PCD-to-PGM conversion
- `relocalizer`: ICP-based relocalization
- `waypoint_planner`: waypoint storage and mission execution
- `px4_offboard_bridge`: MAVROS offboard bridge nodes
- `safety_monitor`: geofence, battery, EKF, and comms watchdog nodes
- `warehouse_utils`: shared helpers
- `px4_mid360`: top-level launch/config package

## Documentation

- `docs/setup_guide.md`: environment setup and basic bringup
- `docs/flight_checklist.md`: pre-flight verification checklist
- `docs/learning_guide.md`: architecture-oriented learning notes
- `docs/learner_full_guide.md`: step-by-step study and full workflow guide for learners
- `docs/code_reading_roadmap.md`: file-by-file source reading order and focus points
- `docs/hands_on_workbook.md`: command-by-command validation workbook
- `docs/inventory_workflow.md`: six-warehouse inventory workflow and capability status

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- PX4 / MAVROS ROS 2 environment
- Livox SDK2 and `livox_ros_driver2`
- FAST-LIO2

Dependency repos are listed in [`dependencies.repos`](dependencies.repos).

## Build

```bash
cd ~/ros2_ws/src
git clone <this-repo> px4_mid360
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Offline Mapping Workflow

1. Replay a recorded bag while FAST-LIO2 and `map_saver` are running:

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

2. Trigger `/map/save` and convert the saved PCD into Nav2-compatible
   `map.pgm` / `map.yaml` files:

```bash
ros2 launch px4_mid360 save_map.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  output_dir:=./maps
```

## Online Mapping Workflow

Bring up MID360, MAVROS, FAST-LIO2 mapping, and live map saving:

```bash
ros2 launch px4_mid360 online_mapping.launch.py \
  map_path:=./maps/warehouse_online_map.pcd
```

This mode is intended for "fly while mapping" operation and can be kept as an
optional workflow alongside the replay-based offline mapping path.

## Inventory Mission Generation

Generate a serpentine "几"-style mission for six side-by-side warehouses:

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

Optionally optimize the warehouse visit order against a saved occupancy map:

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

## Autonomous Flight Workflow

Bring up localization, planning, PX4 bridge, and safety monitors:

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

Start a waypoint mission:

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml
```

The mission launch now automatically calls `/mission/start` after the executor
node is up.

If a pilot manually takes off first and then hands over to offboard mission
execution, you can disable the extra climb step:

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

Create or edit waypoint YAML interactively from RViz:

```bash
ros2 run waypoint_planner interactive_waypoints.py --ros-args \
  -p waypoints_file:=./config/waypoints/warehouse_example.yaml
```

## Notes

- `mission_executor` now checks odometry before starting.
- The control chain now includes a dedicated `local_3d_avoidance` stage:
  `/cmd_vel -> /cmd_vel_safe -> /mavros/setpoint_velocity/cmd_vel`.
- `waypoint_manager` now auto-loads the configured waypoint YAML on startup so
  saved markers are visible without a manual service call.
- Return-to-launch now actually flies back to the recorded home pose before
  descending.
- The six-warehouse inventory workflow can now be generated from
  `config/missions/six_warehouse_layout.yaml`, and `mission_executor` will
  return home automatically after the final waypoint.
- `offboard_controller` is no longer auto-enabled in the main bridge launch, so
  it will not fight with mission execution by default.
- The legacy `z_axis_monitor` is now optional and disabled by default so it
  does not conflict with the 3D avoidance layer.
- Nav2 A* + DWB remain available as a planning reserve for other point-to-point
  autonomous scenarios, while the default inventory workflow uses generated
  mission waypoints plus the 3D local avoidance layer.
- The offline mapping path can now be pinned with a fixed `map_file` so save
  and conversion steps use the same output path.
- Several package launch files were repaired and normalized to avoid syntax
  errors and broken relative paths.
