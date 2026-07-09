# 环境搭建指南

目标环境：

- Ubuntu 22.04
- ROS 2 Humble
- PX4 固件，并启用 `uxrce_dds_client`
- eProsima Micro XRCE-DDS Agent
- Livox SDK2 和 `livox_ros_driver2`
- FAST-LIO2
- EGO-Planner ROS2 branch: `ego-planner-swarm` `ros2_version`
- CycloneDDS: `ros-humble-rmw-cyclonedds-cpp`

## 1. 创建工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repo> px4_mid360
cd ~/ros2_ws
```

## 2. 拉取源码依赖

```bash
vcs import src < src/px4_mid360/dependencies.repos
```

这里会包含 `px4_msgs`、`px4_ros_com`、Livox 驱动、FAST-LIO2、`ego-planner-swarm` ROS2 分支等源码依赖。

注意：`px4_msgs` 必须和 PX4 固件版本匹配。如果编译时报 PX4 消息字段不存在，先对齐 PX4 固件和 `px4_msgs` 分支。

`px4_msgs` 只是消息定义；PX4 真正暴露哪些 `/fmu/in/*` 和 `/fmu/out/*` 话题，由固件里的 `dds_topics.yaml` 决定。实机前一定用下面命令确认：

```bash
ros2 topic list | grep /fmu
```

## 3. 安装 ROS 依赖

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 4. 编译

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 5. 启动 XRCE-DDS Agent

串口示例：

```bash
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

UDP 示例：

```bash
MicroXRCEAgent udp4 -p 8888
```

启动后检查：

```bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_status --once
```

如果 PX4 用命名空间启动，例如 `uxrce_dds_client start -n uav_1`，则项目参数里要把 `px4_namespace` 设置成 `/uav_1`。

## 5.1 启用 EGO-Planner 推荐 DDS

EGO-Planner ROS2 分支 README 建议使用 CycloneDDS，避免默认 FastDDS 下运行卡顿：

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
ros2 doctor --report | grep "RMW middleware"
```

## 6. PX4 参数重点

PX4 侧要保证：

- `uxrce_dds_client` 正常启动。
- EKF2 可接收外部视觉/里程计。
- Offboard failsafe 参数适合室内测试。
- 真实飞行前必须确认遥控器接管逻辑。

项目内参考参数：

```text
config/px4/ekf2_params.txt
config/px4/offboard_params.yaml
src/px4_offboard_bridge/config/offboard_params.yaml
```

## 7. 启动主系统

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  enable_ego_planner_ros2:=true
```

启用后，主系统会启动 EGO-Planner ROS2 分支、`ego_planner_ros2_adapter`、PX4 DDS 桥和 safety filter。

## 8. 常见问题

看得到 `/fmu/out/*` 但节点收不到：

- 检查订阅 QoS 是否使用 `rmw_qos_profile_sensor_data`。
- 检查 ROS domain。
- 检查 XRCE-DDS Agent 是否在跑。

`px4_msgs` 编译失败：

- 对齐 PX4 固件版本和 `px4_msgs` 分支。

飞机方向反了：

- 不要继续飞。
- 检查 FAST-LIO 坐标系、雷达安装方向、ENU/FLU 到 NED/FRD 转换。

定位漂：

- 检查 MID360 外参、FAST-LIO 参数、仓库退化环境、反光和动态障碍。
