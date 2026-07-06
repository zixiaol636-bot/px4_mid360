#ifndef WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_
#define WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace warehouse_utils
{

class FrameConversions
{
public:
  static geometry_msgs::msg::Point flu_to_ned(const geometry_msgs::msg::Point & point);
  static geometry_msgs::msg::Vector3 flu_to_ned(const geometry_msgs::msg::Vector3 & vector);
  static geometry_msgs::msg::Pose flu_to_ned(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::PoseStamped flu_to_ned(
    const geometry_msgs::msg::PoseStamped & pose);
  static geometry_msgs::msg::Twist flu_to_ned(const geometry_msgs::msg::Twist & twist);

  static geometry_msgs::msg::Point ned_to_flu(const geometry_msgs::msg::Point & point);
  static geometry_msgs::msg::Vector3 ned_to_flu(const geometry_msgs::msg::Vector3 & vector);
  static geometry_msgs::msg::Pose ned_to_flu(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::PoseStamped ned_to_flu(
    const geometry_msgs::msg::PoseStamped & pose);
  static geometry_msgs::msg::Twist ned_to_flu(const geometry_msgs::msg::Twist & twist);

  static double yaw_rate_flu_to_ned(double yaw_rate_flu);
  static double yaw_rate_ned_to_flu(double yaw_rate_ned);
};

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS__FRAME_CONVERSIONS_HPP_
