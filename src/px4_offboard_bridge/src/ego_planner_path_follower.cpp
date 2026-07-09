#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class EgoPlannerPathFollower : public rclcpp::Node
{
public:
  EgoPlannerPathFollower() : Node("ego_planner_path_follower")
  {
    declare_parameter<std::string>("planned_path_topic", "/planning/local_path");
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_planned");
    declare_parameter<double>("path_timeout", 0.5);
    declare_parameter<double>("lookahead_distance", 1.0);
    declare_parameter<double>("position_gain", 0.8);
    declare_parameter<double>("yaw_gain", 1.0);
    declare_parameter<double>("max_xy_speed", 1.2);
    declare_parameter<double>("max_z_speed", 0.5);
    declare_parameter<double>("max_yaw_rate", 0.8);
    declare_parameter<double>("publish_rate", 30.0);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("planned_path_topic").as_string(), rclcpp::QoS(5),
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        latest_path_ = *msg;
        last_path_time_ = now();
        has_path_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_out_topic").as_string(), rclcpp::QoS(10));

    const double publish_rate = std::max(1.0, get_parameter("publish_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_command(); });
  }

private:
  void publish_command()
  {
    if (!has_odom_ || !has_path_ ||
      (now() - last_path_time_).seconds() > get_parameter("path_timeout").as_double() ||
      latest_path_.poses.empty())
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    const auto target = select_lookahead_pose();
    const auto & pose = latest_odom_.pose.pose;
    const double dx = target.pose.position.x - pose.position.x;
    const double dy = target.pose.position.y - pose.position.y;
    const double dz = target.pose.position.z - pose.position.z;
    const double yaw = yaw_from_quat(pose.orientation);

    const double vx_world = get_parameter("position_gain").as_double() * dx;
    const double vy_world = get_parameter("position_gain").as_double() * dy;
    const double vz = std::clamp(
      get_parameter("position_gain").as_double() * dz,
      -get_parameter("max_z_speed").as_double(),
      get_parameter("max_z_speed").as_double());

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
    double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

    const double xy_norm = std::hypot(vx_body, vy_body);
    const double max_xy = get_parameter("max_xy_speed").as_double();
    if (xy_norm > max_xy && xy_norm > 1e-6) {
      vx_body *= max_xy / xy_norm;
      vy_body *= max_xy / xy_norm;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx_body;
    cmd.linear.y = vy_body;
    cmd.linear.z = vz;
    cmd.angular.z = std::clamp(
      get_parameter("yaw_gain").as_double() *
      shortest_angular_distance(yaw, desired_yaw_from_path(target)),
      -get_parameter("max_yaw_rate").as_double(),
      get_parameter("max_yaw_rate").as_double());
    cmd_pub_->publish(cmd);
  }

  geometry_msgs::msg::PoseStamped select_lookahead_pose() const
  {
    const auto & current = latest_odom_.pose.pose.position;
    const double lookahead = get_parameter("lookahead_distance").as_double();
    for (const auto & pose : latest_path_.poses) {
      const double distance = std::hypot(
        std::hypot(pose.pose.position.x - current.x, pose.pose.position.y - current.y),
        pose.pose.position.z - current.z);
      if (distance >= lookahead) {
        return pose;
      }
    }
    return latest_path_.poses.back();
  }

  double desired_yaw_from_path(const geometry_msgs::msg::PoseStamped & target) const
  {
    const auto & current = latest_odom_.pose.pose.position;
    const double dx = target.pose.position.x - current.x;
    const double dy = target.pose.position.y - current.y;
    if (std::hypot(dx, dy) < 0.1) {
      return yaw_from_quat(latest_odom_.pose.pose.orientation);
    }
    return std::atan2(dy, dx);
  }

  static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static double shortest_angular_distance(double from, double to)
  {
    double diff = to - from;
    while (diff > M_PI) {
      diff -= 2.0 * M_PI;
    }
    while (diff < -M_PI) {
      diff += 2.0 * M_PI;
    }
    return diff;
  }

  nav_msgs::msg::Path latest_path_;
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};
  bool has_path_{false};
  bool has_odom_{false};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoPlannerPathFollower>());
  rclcpp::shutdown();
  return 0;
}
