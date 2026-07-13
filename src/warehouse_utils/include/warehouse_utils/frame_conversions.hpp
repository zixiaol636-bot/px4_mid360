#ifndef WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_
#define WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// 坐标系转换工具属于 warehouse_utils 模块，避免与其他 ROS 包的同名工具冲突。
namespace warehouse_utils
{

// ROS 侧通常使用 FLU（前、左、上）约定，PX4 侧使用 NED/FRD（北、东、下/
// 前、右、下）约定。本类统一执行 x'=x、y'=-y、z'=-z 的分量转换。
// 所有函数均不修改输入对象，而是返回转换后的新对象。
class FrameConversions
{
public:
  // 将点坐标从 FLU 约定转换为 NED 约定。
  static geometry_msgs::msg::Point flu_to_ned(const geometry_msgs::msg::Point & point);

  // 将三维向量（如线速度）从 FLU 约定转换为 NED 约定。
  static geometry_msgs::msg::Vector3 flu_to_ned(const geometry_msgs::msg::Vector3 & vector);

  // 转换位姿的位置和姿态。姿态会转换为等价的 NED/FRD 四元数。
  static geometry_msgs::msg::Pose flu_to_ned(const geometry_msgs::msg::Pose & pose);

  // 转换带时间戳的位姿；header 原样保留，调用方应自行保证 frame_id 与数据一致。
  static geometry_msgs::msg::PoseStamped flu_to_ned(
    const geometry_msgs::msg::PoseStamped & pose);

  // 转换速度指令：linear 使用坐标轴变换，angular 的 y、z 分量反向。
  static geometry_msgs::msg::Twist flu_to_ned(const geometry_msgs::msg::Twist & twist);

  // NED 到 FLU 与上述变换互为逆变换；本项目的轴变换执行两次即可恢复原值。
  static geometry_msgs::msg::Point ned_to_flu(const geometry_msgs::msg::Point & point);
  static geometry_msgs::msg::Vector3 ned_to_flu(const geometry_msgs::msg::Vector3 & vector);
  static geometry_msgs::msg::Pose ned_to_flu(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::PoseStamped ned_to_flu(
    const geometry_msgs::msg::PoseStamped & pose);
  static geometry_msgs::msg::Twist ned_to_flu(const geometry_msgs::msg::Twist & twist);

  // 偏航角速度在两个约定下正方向相反，因此取负号；单位为弧度每秒。
  static double yaw_rate_flu_to_ned(double yaw_rate_flu);
  static double yaw_rate_ned_to_flu(double yaw_rate_ned);
};

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_
