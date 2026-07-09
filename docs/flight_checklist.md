# 飞行前检查清单

这份清单适用于当前项目默认架构：PX4 XRCE-DDS + MID360 + FAST-LIO + 人工航点盘库 + 3D 局部避障。

注意：地理围栏默认只发布安全状态，不直接抢控制链。如果要做强制接管，建议先增加目标/速度仲裁器，避免任务层和安全层同时发控制输出。

## 1. 硬件

- PX4 飞控供电正常。
- MID360 供电和网线正常。
- 机载电脑与 PX4 串口或以太网连接正常。
- 螺旋桨方向、桨叶、机臂、货架环境安全检查完成。
- 飞手遥控器可随时接管。

## 2. PX4 XRCE-DDS

先启动 XRCE-DDS Agent，然后检查：

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_status --once
```

预期现象：

- 能看到 `/fmu/in/*` 和 `/fmu/out/*` 话题。
- `/fmu/out/vehicle_status` 能输出 `arming_state`、`nav_state` 等字段。

如果只有 topic 但没有数据，优先检查 QoS、Agent、PX4 `uxrce_dds_client` 和 ROS domain。

## 3. MID360

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

预期现象：

- 点云和 IMU 都有稳定频率。
- 点云方向和机体安装方向已经确认。

## 4. FAST-LIO 定位

```bash
ros2 topic echo /odom_filtered --once
ros2 topic hz /cloud_registered
```

预期现象：

- `/odom_filtered` 持续输出位姿。
- `/cloud_registered` 持续输出注册后的点云。
- 缓慢移动飞机时，位姿方向与实际移动方向一致。

## 5. PX4 视觉里程计桥

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

预期现象：

- 能看到由 `/odom_filtered` 转换来的 PX4 视觉里程计消息。
- 实机前必须低速验证 ENU/FLU 到 NED/FRD 的方向转换。

## 6. PX4 状态桥

```bash
ros2 topic echo /px4_native/status --once
ros2 topic echo /px4_native/armed --once
```

预期现象：

- `/px4_native/status` 有 `arming_state`、`nav_state`、`failsafe`。
- `/px4_native/armed` 能反映 PX4 解锁状态。

## 7. Offboard 心跳

启动主系统后检查：

```bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

预期现象：

- 两个话题稳定发布。
- 频率接近参数中设置的 Offboard 发布频率。

## 8. 3D 局部避障

```bash
ros2 topic echo /cmd_vel_safe --once
```

实验方法：

- 先发一个很小的 `/cmd_vel_planned` 前进速度。
- 在 MID360 前方安全距离内放入障碍物。
- 观察 `/drone_0_planning/pos_cmd`、`/cmd_vel_planned` 和 `/cmd_vel_safe`：EGO-Planner ROS2 负责绕障轨迹，安全过滤器只负责危险时减速或刹停。

## 9. 任务层

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
ros2 topic echo /planning/goal_pose --once
```

预期现象：

- 任务开始后 `/planning/goal_pose` 有输出。
- 飞机接近航点后会切到下一个航点。
- 最后一个仓库完成后进入 RTL 返回起飞点。

## 10. 实机底线

- 第一次只做架空或拆桨测试。
- 第二次只做低速、低高度、短距离测试。
- 第三次再做单仓测试。
- 六仓连续盘库必须在单仓和双仓成功后再做。
- 任意方向控制反了，立即停止，不要靠参数硬飞。
