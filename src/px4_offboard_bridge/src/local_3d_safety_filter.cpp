#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Local3DSafetyFilter : public rclcpp::Node
{
public:
  Local3DSafetyFilter() : Node("local_3d_safety_filter")
  {
    declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel_planned");
    declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_safe");
    declare_parameter<std::string>("pointcloud_topic", "/cloud_registered");
    declare_parameter<std::string>("pointcloud_frame_mode", "world");
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("stop_status_topic", "/safety/obstacle_stop");
    declare_parameter<double>("cmd_timeout", 0.5);
    declare_parameter<double>("cloud_timeout", 0.8);
    declare_parameter<double>("danger_distance", 0.8);
    declare_parameter<double>("caution_distance", 2.5);
    declare_parameter<double>("corridor_radius_xy", 0.7);
    declare_parameter<double>("corridor_radius_z", 0.6);
    declare_parameter<double>("max_xy_speed", 1.2);
    declare_parameter<double>("max_z_speed", 0.5);
    declare_parameter<double>("publish_rate", 30.0);
    declare_parameter<bool>("stop_on_stale_cloud", true);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_in_topic").as_string(), rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        last_cmd_time_ = now();
        has_cmd_ = true;
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("pointcloud_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_cloud_ = *msg;
        last_cloud_time_ = now();
        has_cloud_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_out_topic").as_string(), rclcpp::QoS(10));
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("stop_status_topic").as_string(), rclcpp::QoS(1).transient_local());

    const double publish_rate = std::max(1.0, get_parameter("publish_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_filtered_cmd(); });
  }

private:
  void publish_filtered_cmd()
  {
    if (!has_cmd_ ||
      (now() - last_cmd_time_).seconds() > get_parameter("cmd_timeout").as_double())
    {
      publish_stop(false);
      return;
    }

    const bool fresh_cloud = has_cloud_ &&
      (now() - last_cloud_time_).seconds() <= get_parameter("cloud_timeout").as_double();
    const bool needs_odom = get_parameter("pointcloud_frame_mode").as_string() != "body";
    if (needs_odom && !has_odom_) {
      publish_stop(true);
      return;
    }
    if (!fresh_cloud && get_parameter("stop_on_stale_cloud").as_bool()) {
      publish_stop(true);
      return;
    }

    geometry_msgs::msg::Twist cmd = clamp_cmd(latest_cmd_);
    const double speed = std::hypot(std::hypot(cmd.linear.x, cmd.linear.y), cmd.linear.z);
    if (!fresh_cloud || speed < 1e-4) {
      cmd_pub_->publish(cmd);
      publish_stop_status(false);
      return;
    }

    const double closest = closest_obstacle_along_cmd(cmd);
    const double danger = get_parameter("danger_distance").as_double();
    const double caution = std::max(danger + 0.1, get_parameter("caution_distance").as_double());

    if (closest <= danger) {
      publish_stop(true);
      return;
    }

    if (closest < caution) {
      const double scale = std::clamp((closest - danger) / (caution - danger), 0.0, 1.0);
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
      cmd.linear.z *= scale;
    }

    cmd_pub_->publish(cmd);
    publish_stop_status(false);
  }

  double closest_obstacle_along_cmd(const geometry_msgs::msg::Twist & cmd) const
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(latest_cloud_, cloud);
    if (cloud.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    tf2::Vector3 direction(cmd.linear.x, cmd.linear.y, cmd.linear.z);
    if (direction.length2() < 1e-8) {
      return std::numeric_limits<double>::infinity();
    }
    direction.normalize();

    double closest = std::numeric_limits<double>::infinity();
    const double caution = get_parameter("caution_distance").as_double();
    const double radius_xy = get_parameter("corridor_radius_xy").as_double();
    const double radius_z = get_parameter("corridor_radius_z").as_double();

    for (const auto & raw_point : cloud.points) {
      const auto point = point_in_body_frame(raw_point);
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
        continue;
      }

      const double along = point.dot(direction);
      if (along <= 0.0 || along > caution) {
        continue;
      }

      const tf2::Vector3 lateral = point - direction * along;
      if (std::hypot(lateral.x(), lateral.y()) <= radius_xy &&
        std::abs(lateral.z()) <= radius_z)
      {
        closest = std::min(closest, along);
      }
    }

    return closest;
  }

  tf2::Vector3 point_in_body_frame(const pcl::PointXYZ & point) const
  {
    if (get_parameter("pointcloud_frame_mode").as_string() == "body") {
      return tf2::Vector3(point.x, point.y, point.z);
    }

    tf2::Quaternion q_body_to_world;
    tf2::fromMsg(latest_odom_.pose.pose.orientation, q_body_to_world);
    q_body_to_world.normalize();

    const auto & origin = latest_odom_.pose.pose.position;
    const tf2::Vector3 world_delta(
      point.x - origin.x,
      point.y - origin.y,
      point.z - origin.z);
    return tf2::quatRotate(q_body_to_world.inverse(), world_delta);
  }

  geometry_msgs::msg::Twist clamp_cmd(const geometry_msgs::msg::Twist & input) const
  {
    geometry_msgs::msg::Twist cmd = input;
    const double max_xy = get_parameter("max_xy_speed").as_double();
    const double xy = std::hypot(cmd.linear.x, cmd.linear.y);
    if (xy > max_xy && xy > 1e-6) {
      cmd.linear.x *= max_xy / xy;
      cmd.linear.y *= max_xy / xy;
    }
    cmd.linear.z = std::clamp(
      cmd.linear.z,
      -get_parameter("max_z_speed").as_double(),
      get_parameter("max_z_speed").as_double());
    return cmd;
  }

  void publish_stop(bool obstacle_stop)
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    publish_stop_status(obstacle_stop);
  }

  void publish_stop_status(bool obstacle_stop)
  {
    std_msgs::msg::Bool msg;
    msg.data = obstacle_stop;
    stop_pub_->publish(msg);
  }

  geometry_msgs::msg::Twist latest_cmd_;
  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::PointCloud2 latest_cloud_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};
  bool has_cloud_{false};
  bool has_odom_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Local3DSafetyFilter>());
  rclcpp::shutdown();
  return 0;
}
