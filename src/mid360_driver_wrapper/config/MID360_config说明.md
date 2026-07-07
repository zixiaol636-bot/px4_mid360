# MID360_config.json 字段说明

> 此文件是 Livox 官方驱动 `livox_ros_driver2` 的配置文件，**JSON 格式不支持注释**，所以分开说明。

## device_net_info — MID360 自身端口（不动）
| 字段 | 值 | 说明 |
|------|-----|------|
| cmd_data_port | 56000 | 命令端口 |
| push_msg_port | 0 | 推送端口（MID360 不用） |
| point_data_port | 56001 | 点云数据端口 |
| imu_data_port | 56002 | IMU 数据端口 |
| log_data_port | 56003 | 日志端口 |

## host_net_info — 机载电脑 IP（按实际改）
| 字段 | 值 | 说明 |
|------|-----|------|
| cmd_data_ip | 192.168.1.50 | 机载电脑 IP |
| point_data_ip | 192.168.1.50 | 接收点云的 IP |
| imu_data_ip | 192.168.1.50 | 接收 IMU 的 IP |
| log_data_ip | 192.168.1.50 | 接收日志的 IP |

> ⚠️ `192.168.1.50` 是机载电脑的静态 IP，如果你的机载电脑 IP 不同，**改这里**。

## lidar_net_info — MID360 自身 IP（不动）
| 字段 | 值 | 说明 |
|------|-----|------|
| cmd_data_ip | 192.168.1.152 | MID360 默认 IP |

## lidar_configs — 雷达参数（不动）
| 字段 | 值 | 说明 |
|------|-----|------|
| lidar_type | 1 | 1 = Livox 非重复扫描（MID360） |
| pcl_data_type | 1 | 输出 PointCloud2 格式 |
| pattern_mode | 0 | 扫描模式（0 = 默认） |
| enable_imu | false | MID360 自带 IMU 不通过此字段启用 |
| extrinsic_parameter | 全 0 | LiDAR→机身 外参，在 FAST-LIO2 配，这里不动 |
