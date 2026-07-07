/**
 * @file frame_conversions.hpp
 * @brief ROS FLU 坐标系 与 PX4 NED 坐标系 之间的双向转换
 *
 * ROS 使用 FLU:  x=前  y=左  z=上
 * PX4 使用 NED:  x=北  y=东  z=下
 *
 * 不转换直接发给 PX4 → 方向全乱，飞机失控。
 * 所有与 PX4 交换位姿/速度的节点都必须经过这里。
 */

#ifndef WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_
#define WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_

#include <geometry_msgs/msg/point.hpp>          // (x, y, z) 纯坐标点
#include <geometry_msgs/msg/pose.hpp>           // Point + Quaternion 位姿
#include <geometry_msgs/msg/pose_stamped.hpp>   // Pose + header(时间戳+坐标系) 最常用
#include <geometry_msgs/msg/quaternion.hpp>     // (x, y, z, w) 四元数，内部转换用
#include <geometry_msgs/msg/twist.hpp>          // linear(Vector3) + angular(Vector3) 速度指令
#include <geometry_msgs/msg/vector3.hpp>        // (x, y, z) 有方向的物理量，如速度/加速度

namespace warehouse_utils
{

/// 纯静态工具类，提供 FLU ↔ NED 坐标系转换，无需实例化
class FrameConversions
{
public:
  // ===== FLU → NED（ROS → PX4）=====

  /// 点坐标: (x,y,z) → (x, -y, -z)
  static geometry_msgs::msg::Point flu_to_ned(const geometry_msgs::msg::Point & point);

  /// 向量: 同上，x 不变，y 和 z 取反
  static geometry_msgs::msg::Vector3 flu_to_ned(const geometry_msgs::msg::Vector3 & vector);

  /// 位姿: 位置同 Point，朝向通过 RPY 翻转 pitch 和 yaw 后重建四元数
  static geometry_msgs::msg::Pose flu_to_ned(const geometry_msgs::msg::Pose & pose);

  /// 带时间戳位姿（最常用），内部委托给 Pose 版本，保留 header
  static geometry_msgs::msg::PoseStamped flu_to_ned(
    const geometry_msgs::msg::PoseStamped & pose);

  /// 速度指令: 线速度同 Vector3，角速度的 y(z) 取反
  static geometry_msgs::msg::Twist flu_to_ned(const geometry_msgs::msg::Twist & twist);

  // ===== NED → FLU（PX4 → ROS）=====

  static geometry_msgs::msg::Point ned_to_flu(const geometry_msgs::msg::Point & point);
  static geometry_msgs::msg::Vector3 ned_to_flu(const geometry_msgs::msg::Vector3 & vector);
  static geometry_msgs::msg::Pose ned_to_flu(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::PoseStamped ned_to_flu(const geometry_msgs::msg::PoseStamped & pose);
  static geometry_msgs::msg::Twist ned_to_flu(const geometry_msgs::msg::Twist & twist);

  // ===== 偏航角速度 ====

  /// FLU yaw 正=CCW，NED yaw 正=CW，所以取反
  static double yaw_rate_flu_to_ned(double yaw_rate_flu);
  static double yaw_rate_ned_to_flu(double yaw_rate_ned);
};

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_
