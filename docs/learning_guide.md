# 学习指南

这个项目可以按四层理解：感知建图、任务规划、避障安全、PX4 执行。

## 1. 感知建图层

主要输入：

```text
MID360 -> livox_ros_driver2 -> FAST-LIO2
```

主要输出：

```text
/odom_filtered
/cloud_registered
```

你先学这层，是因为室内无人机没有 GPS，后面所有航点飞行都依赖 FAST-LIO 的定位。

## 2. 任务规划层

盘库主流程不是让无人机完全自由探索，而是人工设置或脚本生成航点。

典型链路：

```text
six_warehouse_layout.yaml
  -> generate_inventory_mission.py
  -> six_warehouse_inventory.yaml
  -> mission_executor
  -> /planning/goal_pose
```

为什么任务层要自己算距离：因为人工航点任务需要判断“我离当前航点还有多远、该不该切下一个航点、是否超时、是否进入悬停或返航”。这不是全局规划器，它是航点任务状态机。名义 `/cmd_vel` 只保留作调试/兼容输出，实际主链路通过 `/planning/goal_pose` 交给 EGO-Planner。

## 3. 避障安全层

主要节点：

```text
ego_planner_path_follower
ego_planner_ros2_adapter
local_3d_safety_filter
```

链路：

```text
/planning/goal_pose -> ego_planner_ros2_adapter -> EGO-Planner ROS2 -> /drone_0_planning/pos_cmd -> /cmd_vel_planned -> /cmd_vel_safe
```

这层做的是 3D 局部轨迹规划和最后安全过滤。EGO-Planner ROS2 负责绕障轨迹，`ego_planner_ros2_adapter` 负责话题和速度接口转换，`local_3d_safety_filter` 只负责危险时减速或停车。

当前默认把 FAST-LIO 的 `/cloud_registered` 当作世界系点云处理，利用 `/odom_filtered` 转到无人机机体系后再判断障碍方向。这样比直接假设点云 x 轴就是机头前方更适合实际 FAST-LIO 输出。

## 4. PX4 执行层

主要节点：

```text
px4_visual_odometry_bridge
px4_cmdvel_bridge
px4_status_bridge
```

链路：

```text
/cmd_vel_safe -> /fmu/in/trajectory_setpoint
/odom_filtered -> /fmu/in/vehicle_visual_odometry
/fmu/out/vehicle_status -> /px4_native/status
```

PX4 通过 XRCE-DDS 和 ROS 2 通信，所以必须有 `px4_msgs`。

## 5. 推荐学习顺序

1. 先读 `README.md`，知道系统能干什么。
2. 再读 `docs/px4_ros2_dds_guide.md`，理解 PX4 DDS 和 QoS。
3. 再读 `docs/code_reading_roadmap.md`，按文件看代码。
4. 跟着 `docs/hands_on_workbook.md` 做实验。
5. 最后读 `docs/inventory_workflow.md`，理解六仓盘库业务流程。

## 6. 你要重点掌握的概念

- ROS ENU/FLU 和 PX4 NED/FRD 坐标转换。
- PX4 Offboard 需要持续心跳，不是发一次目标点就结束。
- `/fmu/out/*` 订阅要用 PX4 兼容 QoS。
- 任务层负责航点状态和目标发布，EGO-Planner 负责 3D 局部轨迹，安全过滤器负责最后减速/停车，PX4 桥负责和飞控通信。
- A* 优化是技术储备，不是默认盘库主流程。
