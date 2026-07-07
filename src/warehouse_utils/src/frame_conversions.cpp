/**
 * @file frame_conversions.cpp
 * @brief FLU ↔ NED 坐标系转换的实现
 *
 * 转换规则：
 *   FLU (x=前 y=左 z=上)  →  NED (x=北 y=东 z=下)
 *
 *   Point/Vector3:  (x, y, z) → (x, -y, -z)     ← y 和 z 取反
 *   Pose:           位置同上，朝向: RPY → 翻转 pitch 和 yaw → 重建四元数
 *   Twist:          线速度同 Vector3，角速度 pitch(y) 和 yaw(z) 取反
 *   yaw_rate:       取反（FLU 正=CCW，NED 正=CW）
 */

#include "warehouse_utils/frame_conversions.hpp"

#include <tf2/LinearMath/Matrix3x3.h>       // getRPY() 提取欧拉角
#include <tf2/LinearMath/Quaternion.h>      // setRPY() 重建四元数
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // fromMsg / toMsg 消息转换

namespace warehouse_utils
{

// 匿名命名空间：这两个四元数转换函数只在本文件内部使用，不对外暴露
namespace
{

// FLU 四元数 → NED 四元数
// 策略：先转 RPY → 翻转 pitch 和 yaw → 重建四元数（不直接改 x/y/z/w 分量，容易出错）
geometry_msgs::msg::Quaternion convert_quaternion_flu_to_ned(
  const geometry_msgs::msg::Quaternion & q_in)
{
  tf2::Quaternion q_tf;
  tf2::fromMsg(q_in, q_tf);           // ROS 消息 → tf2 内部四元数

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);  // 提取欧拉角

  tf2::Quaternion q_out;
  q_out.setRPY(roll, -pitch, -yaw);    // pitch 和 yaw 翻转后重建
  return tf2::toMsg(q_out);
}

// NED 四元数 → FLU 四元数（与上面过程对称）
geometry_msgs::msg::Quaternion convert_quaternion_ned_to_flu(
  const geometry_msgs::msg::Quaternion & q_in)
{
  tf2::Quaternion q_tf;
  tf2::fromMsg(q_in, q_tf);

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

  tf2::Quaternion q_out;
  q_out.setRPY(roll, -pitch, -yaw);
  return tf2::toMsg(q_out);
}

}  // namespace

// ===== FLU → NED（ROS → PX4）=====

// (x, y, z) → (x, -y, -z)
geometry_msgs::msg::Point FrameConversions::flu_to_ned(const geometry_msgs::msg::Point & point)
{
  geometry_msgs::msg::Point out;
  out.x = point.x;
  out.y = -point.y;
  out.z = -point.z;
  return out;
}

// 同 Point，x 不变，y 和 z 取反
geometry_msgs::msg::Vector3 FrameConversions::flu_to_ned(
  const geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Vector3 out;
  out.x = vector.x;
  out.y = -vector.y;
  out.z = -vector.z;
  return out;
}

// 位置调 flu_to_ned(Point)，朝向调 convert_quaternion_flu_to_ned
geometry_msgs::msg::Pose FrameConversions::flu_to_ned(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose out;
  out.position = flu_to_ned(pose.position);
  out.orientation = convert_quaternion_flu_to_ned(pose.orientation);
  return out;
}

// 委托给 Pose 版本，header（frame_id + 时间戳）原样保留
geometry_msgs::msg::PoseStamped FrameConversions::flu_to_ned(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped out = pose;
  out.pose = flu_to_ned(pose.pose);
  return out;
}

// 线速度同 Vector3；角速度: roll 不变，pitch(y) 和 yaw(z) 取反
geometry_msgs::msg::Twist FrameConversions::flu_to_ned(const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::Twist out;
  out.linear = flu_to_ned(twist.linear);
  out.angular.x = twist.angular.x;
  out.angular.y = -twist.angular.y;
  out.angular.z = -twist.angular.z;
  return out;
}

// ===== NED → FLU（PX4 → ROS）=====

geometry_msgs::msg::Point FrameConversions::ned_to_flu(const geometry_msgs::msg::Point & point)
{
  geometry_msgs::msg::Point out;
  out.x = point.x;
  out.y = -point.y;
  out.z = -point.z;
  return out;
}

geometry_msgs::msg::Vector3 FrameConversions::ned_to_flu(
  const geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Vector3 out;
  out.x = vector.x;
  out.y = -vector.y;
  out.z = -vector.z;
  return out;
}

geometry_msgs::msg::Pose FrameConversions::ned_to_flu(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose out;
  out.position = ned_to_flu(pose.position);
  out.orientation = convert_quaternion_ned_to_flu(pose.orientation);
  return out;
}

geometry_msgs::msg::PoseStamped FrameConversions::ned_to_flu(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped out = pose;
  out.pose = ned_to_flu(pose.pose);
  return out;
}

geometry_msgs::msg::Twist FrameConversions::ned_to_flu(const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::Twist out;
  out.linear = ned_to_flu(twist.linear);
  out.angular.x = twist.angular.x;
  out.angular.y = -twist.angular.y;
  out.angular.z = -twist.angular.z;
  return out;
}

// ===== 偏航角速度 ====

// FLU yaw 正=CCW，NED yaw 正=CW，取反即可
double FrameConversions::yaw_rate_flu_to_ned(double yaw_rate_flu)
{
  return -yaw_rate_flu;
}

double FrameConversions::yaw_rate_ned_to_flu(double yaw_rate_ned)
{
  return -yaw_rate_ned;
}

}  // namespace warehouse_utils
