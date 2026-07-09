#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class EgoPlannerRos2Adapter : public rclcpp::Node
{
public:
  EgoPlannerRos2Adapter() : Node("ego_planner_ros2_adapter")
  {
    declare_parameter<std::string>("odom_in_topic", "/odom_filtered");
    declare_parameter<std::string>("cloud_in_topic", "/cloud_registered");
    declare_parameter<std::string>("goal_in_topic", "/planning/goal_pose");
    declare_parameter<std::string>("ego_odom_out_topic", "/drone_0_odom_filtered");
    declare_parameter<std::string>("ego_cloud_out_topic", "/drone_0_cloud_registered");
    declare_parameter<std::string>("ego_goal_out_topic", "/move_base_simple/goal");
    declare_parameter<std::string>("ego_position_cmd_topic", "/drone_0_planning/pos_cmd");
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_planned");
    declare_parameter<double>("publish_rate", 50.0);
    declare_parameter<double>("position_gain", 0.8);
    declare_parameter<double>("yaw_gain", 1.0);
    declare_parameter<double>("cmd_timeout", 0.3);
    declare_parameter<double>("max_xy_speed", 1.2);
    declare_parameter<double>("max_z_speed", 0.5);
    declare_parameter<double>("max_yaw_rate", 0.8);
    declare_parameter<bool>("stop_without_odom", true);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_in_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
        ego_odom_pub_->publish(*msg);
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("cloud_in_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        ego_cloud_pub_->publish(*msg);
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_in_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // EGO-Planner ROS2 listens to /move_base_simple/goal in manual target mode.
        ego_goal_pub_->publish(*msg);
      });

    position_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      get_parameter("ego_position_cmd_topic").as_string(), rclcpp::QoS(10),
      [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
        latest_position_cmd_ = *msg;
        last_cmd_time_ = now();
        has_position_cmd_ = true;
      });

    ego_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      get_parameter("ego_odom_out_topic").as_string(), rclcpp::QoS(10));
    ego_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("ego_cloud_out_topic").as_string(), rclcpp::SensorDataQoS());
    ego_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("ego_goal_out_topic").as_string(), rclcpp::QoS(1).transient_local());
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_out_topic").as_string(), rclcpp::QoS(10));

    const double publish_rate = std::max(1.0, get_parameter("publish_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_cmd_vel(); });
  }

private:
  void publish_cmd_vel()
  {
    const bool fresh_cmd = has_position_cmd_ &&
      (now() - last_cmd_time_).seconds() <= get_parameter("cmd_timeout").as_double();
    if (!fresh_cmd ||
      latest_position_cmd_.trajectory_flag !=
      quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY ||
      (!has_odom_ && get_parameter("stop_without_odom").as_bool()))
    {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
      return;
    }

    const auto & cmd = latest_position_cmd_;
    double vx_world = cmd.velocity.x;
    double vy_world = cmd.velocity.y;
    double vz = cmd.velocity.z;
    double yaw_rate = cmd.yaw_dot;

    if (has_odom_) {
      const auto & pose = latest_odom_.pose.pose;
      vx_world += get_parameter("position_gain").as_double() *
        (cmd.position.x - pose.position.x);
      vy_world += get_parameter("position_gain").as_double() *
        (cmd.position.y - pose.position.y);
      vz += get_parameter("position_gain").as_double() *
        (cmd.position.z - pose.position.z);
      yaw_rate += get_parameter("yaw_gain").as_double() *
        shortest_angular_distance(yaw_from_quat(pose.orientation), cmd.yaw);
    }

    const double yaw = has_odom_ ? yaw_from_quat(latest_odom_.pose.pose.orientation) : 0.0;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    geometry_msgs::msg::Twist out;
    out.linear.x = cos_yaw * vx_world + sin_yaw * vy_world;
    out.linear.y = -sin_yaw * vx_world + cos_yaw * vy_world;
    out.linear.z = vz;

    const double xy = std::hypot(out.linear.x, out.linear.y);
    const double max_xy = get_parameter("max_xy_speed").as_double();
    if (xy > max_xy && xy > 1e-6) {
      out.linear.x *= max_xy / xy;
      out.linear.y *= max_xy / xy;
    }

    out.linear.z = std::clamp(
      out.linear.z,
      -get_parameter("max_z_speed").as_double(),
      get_parameter("max_z_speed").as_double());
    out.angular.z = std::clamp(
      yaw_rate,
      -get_parameter("max_yaw_rate").as_double(),
      get_parameter("max_yaw_rate").as_double());

    cmd_vel_pub_->publish(out);
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

  nav_msgs::msg::Odometry latest_odom_;
  quadrotor_msgs::msg::PositionCommand latest_position_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_odom_{false};
  bool has_position_cmd_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ego_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ego_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ego_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoPlannerRos2Adapter>());
  rclcpp::shutdown();
  return 0;
}
