# EGO-Planner ROS2 Integration Guide

This project integrates `ZJU-FAST-Lab/ego-planner-swarm` on the `ros2_version`
branch as the real 3D local trajectory planner.

The dependency is listed in `dependencies.repos`:

```bash
vcs import src < src/px4_mid360/dependencies.repos
```

The upstream ROS2 branch recommends CycloneDDS because their default FastDDS
setup can lag:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
ros2 doctor --report | grep "RMW middleware"
```

## Runtime Chain

```text
mission_executor
  -> /planning/goal_pose
  -> ego_planner_ros2_adapter
  -> /move_base_simple/goal
  -> EGO-Planner ROS2
  -> /drone_0_planning/pos_cmd
  -> ego_planner_ros2_adapter
  -> /cmd_vel_planned
  -> local_3d_safety_filter
  -> /cmd_vel_safe
  -> px4_cmdvel_bridge
  -> PX4
```

The adapter also relays:

```text
/odom_filtered      -> /drone_0_odom_filtered
/cloud_registered   -> /drone_0_cloud_registered
```

## Launch

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  enable_ego_planner_ros2:=true
```

When this option is enabled, `bringup_all.launch.py` disables the internal
`ego_planner_path_follower`, so only the ROS2 EGO adapter publishes
`/cmd_vel_planned`.

## Important Upstream Limitation

In the current upstream `ros2_version`, EGO's manual target callback subscribes
to `/move_base_simple/goal`, but the source sets target z to `1.0` instead of
using `msg->pose.position.z`.

For real warehouse flight at configurable heights, patch this line in
`ego_replan_fsm.cpp` after importing the dependency:

```cpp
Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
```

to:

```cpp
Eigen::Vector3d end_wp(
  msg->pose.position.x,
  msg->pose.position.y,
  msg->pose.position.z);
```

## Why Keep The Safety Filter

EGO-Planner generates a feasible local 3D trajectory. The safety filter is still
required because real flights can hit stale point clouds, localization jumps,
delayed trajectories, or unexpected obstacles.

`local_3d_safety_filter` does not plan around obstacles. It only slows down or
stops the final velocity command if the current point cloud shows an immediate
collision risk.

## Flight Test Order

1. Replay a mapping bag and verify `/odom_filtered` and `/cloud_registered`.
2. Start with `enable_ego_planner_ros2:=true`.
3. Confirm `/move_base_simple/goal`, `/drone_0_odom_filtered`, and `/drone_0_cloud_registered` update.
4. Confirm `/drone_0_planning/pos_cmd` updates after a goal.
5. Confirm `/cmd_vel_planned` and `/cmd_vel_safe` update.
6. Confirm `px4_cmdvel_bridge` publishes `/fmu/in/trajectory_setpoint`.
7. Only then connect PX4 Offboard at low speed, with propellers removed or in a protected test setup first.
