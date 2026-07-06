# 飞行前检查清单

## 1. 环境

- [ ] 作业区域已清空人员和杂物
- [ ] 起降区平整，LiDAR 视野无遮挡
- [ ] 已确认室内 GPS 不可用，任务依赖 LiDAR + 视觉/里程计链路
- [ ] 返航点附近没有吊物、横梁、低垂管线

## 2. 硬件

- [ ] MID360 通电正常
- [ ] 飞控通电正常
- [ ] 机载电脑供电稳定
- [ ] 以太网、USB、串口线缆固定可靠
- [ ] 电池电量充足
- [ ] 螺旋桨和机架紧固

## 3. 软件

- [ ] `source ~/ros2_ws/install/setup.bash`
- [ ] `ros2 topic list` 能看到核心话题
- [ ] `/livox/lidar` 正常发布
- [ ] `/livox/imu` 正常发布
- [ ] `/mavros/state` 正常发布
- [ ] `/odom_filtered` 正常发布
- [ ] `/map_cloud` 或地图加载流程正常

## 4. 参数与地图

- [ ] `config/fast_lio/mid360_localization.yaml` 已确认
- [ ] 当前使用的 `.pcd` 地图正确
- [ ] 当前使用的 `map.yaml` 与 `.pcd` 地图匹配
- [ ] 航点文件已确认是本次任务版本
- [ ] `offboard_params.yaml` 已确认速度和加速度限制

## 5. 启动顺序

- [ ] 先启动定位或全系统
- [ ] 再确认 `/mavros/vision_pose/pose` 正常
- [ ] 再确认 `/cmd_vel -> /cmd_vel_safe -> /mavros/setpoint_velocity/cmd_vel` 链路正常
- [ ] 再进入 OFFBOARD / 自动任务

## 6. 起飞前 ROS 自检

推荐至少执行这些检查：

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /odom_filtered --once
ros2 topic echo /mavros/vision_pose/pose --once
```

如果跑全系统：

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

如果执行任务：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml
```

## 7. 飞控状态

- [ ] MAVROS 已连接飞控
- [ ] 飞控姿态估计稳定
- [ ] 本地位置估计稳定
- [ ] 无明显 EKF 报警
- [ ] 电池状态正常

## 8. 风险动作前再确认

- [ ] 手上有随时切回手动或急停的方案
- [ ] 任务初次验证时，先低高度、低速度
- [ ] 第一次上机时不要同时验证太多变量

## 9. 首飞建议

第一次真实测试建议顺序：

1. 只起系统，不起飞。
2. 只验证定位和地图对齐。
3. 手动起飞，验证视觉位姿与 LiDAR 链路。
4. 短距离小范围自动速度控制。
5. 最后再跑完整航点任务。
