#include "warehouse_utils/frame_conversions.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace warehouse_utils
{
namespace
{

geometry_msgs::msg::Quaternion mirror_pitch_yaw(
  const geometry_msgs::msg::Quaternion & q_in)
{
  tf2::Quaternion q_tf;
  tf2::fromMsg(q_in, q_tf);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

  tf2::Quaternion q_out;
  q_out.setRPY(roll, -pitch, -yaw);
  q_out.normalize();
  return tf2::toMsg(q_out);
}

}  // namespace

geometry_msgs::msg::Point FrameConversions::flu_to_ned(
  const geometry_msgs::msg::Point & point)
{
  geometry_msgs::msg::Point out;
  out.x = point.x;
  out.y = -point.y;
  out.z = -point.z;
  return out;
}

geometry_msgs::msg::Vector3 FrameConversions::flu_to_ned(
  const geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Vector3 out;
  out.x = vector.x;
  out.y = -vector.y;
  out.z = -vector.z;
  return out;
}

geometry_msgs::msg::Pose FrameConversions::flu_to_ned(
  const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose out;
  out.position = flu_to_ned(pose.position);
  out.orientation = mirror_pitch_yaw(pose.orientation);
  return out;
}

geometry_msgs::msg::PoseStamped FrameConversions::flu_to_ned(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped out = pose;
  out.pose = flu_to_ned(pose.pose);
  return out;
}

geometry_msgs::msg::Twist FrameConversions::flu_to_ned(
  const geometry_msgs::msg::Twist & twist)
{
  geometry_msgs::msg::Twist out;
  out.linear = flu_to_ned(twist.linear);
  out.angular.x = twist.angular.x;
  out.angular.y = -twist.angular.y;
  out.angular.z = -twist.angular.z;
  return out;
}

geometry_msgs::msg::Point FrameConversions::ned_to_flu(
  const geometry_msgs::msg::Point & point)
{
  return flu_to_ned(point);
}

geometry_msgs::msg::Vector3 FrameConversions::ned_to_flu(
  const geometry_msgs::msg::Vector3 & vector)
{
  return flu_to_ned(vector);
}

geometry_msgs::msg::Pose FrameConversions::ned_to_flu(
  const geometry_msgs::msg::Pose & pose)
{
  return flu_to_ned(pose);
}

geometry_msgs::msg::PoseStamped FrameConversions::ned_to_flu(
  const geometry_msgs::msg::PoseStamped & pose)
{
  return flu_to_ned(pose);
}

geometry_msgs::msg::Twist FrameConversions::ned_to_flu(
  const geometry_msgs::msg::Twist & twist)
{
  return flu_to_ned(twist);
}

double FrameConversions::yaw_rate_flu_to_ned(double yaw_rate_flu)
{
  return -yaw_rate_flu;
}

double FrameConversions::yaw_rate_ned_to_flu(double yaw_rate_ned)
{
  return -yaw_rate_ned;
}

}  // namespace warehouse_utils
