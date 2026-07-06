#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

class Local3DAvoidance : public rclcpp::Node
{
public:
  Local3DAvoidance()
  : Node("local_3d_avoidance"),
    last_desired_time_(this->now()),
    last_cloud_time_(this->now())
  {
    this->declare_parameter<std::string>("cmd_vel_in_topic", "/cmd_vel");
    this->declare_parameter<std::string>("cmd_vel_out_topic", "/cmd_vel_safe");
    this->declare_parameter<std::string>("pointcloud_topic", "/cloud_registered");
    this->declare_parameter<std::string>("odom_topic", "/odom_filtered");
    this->declare_parameter<double>("publish_rate", 20.0);
    this->declare_parameter<double>("input_timeout", 0.5);
    this->declare_parameter<double>("cloud_timeout", 0.5);
    this->declare_parameter<bool>("cloud_in_body_frame", false);
    this->declare_parameter<int>("max_points_to_process", 4000);
    this->declare_parameter<double>("forward_influence_distance", 4.0);
    this->declare_parameter<double>("rear_influence_distance", 1.0);
    this->declare_parameter<double>("side_influence_distance", 2.5);
    this->declare_parameter<double>("vertical_influence_distance", 2.0);
    this->declare_parameter<double>("safety_distance_xy", 1.2);
    this->declare_parameter<double>("safety_distance_z", 0.8);
    this->declare_parameter<double>("hard_stop_distance", 0.6);
    this->declare_parameter<double>("repulsion_gain_xy", 1.4);
    this->declare_parameter<double>("repulsion_gain_z", 1.1);
    this->declare_parameter<double>("lateral_escape_gain", 0.8);
    this->declare_parameter<double>("vertical_escape_gain", 0.6);
    this->declare_parameter<double>("smoothing_alpha", 0.35);
    this->declare_parameter<double>("max_output_xy", 1.5);
    this->declare_parameter<double>("max_output_z", 0.6);
    this->declare_parameter<double>("max_output_yaw_rate", 1.0);

    const auto cmd_vel_in_topic = this->get_parameter("cmd_vel_in_topic").as_string();
    const auto cmd_vel_out_topic = this->get_parameter("cmd_vel_out_topic").as_string();
    const auto pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    const auto odom_topic = this->get_parameter("odom_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    desired_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_in_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { desired_callback(msg); });

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(msg); });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_avoided_cmd(); });

    RCLCPP_INFO(
      this->get_logger(),
      "Local3DAvoidance: %s -> %s using cloud %s",
      cmd_vel_in_topic.c_str(),
      cmd_vel_out_topic.c_str(),
      pointcloud_topic.c_str());
  }

private:
  struct ObstacleSummary
  {
    tf2::Vector3 repulsion{0.0, 0.0, 0.0};
    double front_clearance{std::numeric_limits<double>::infinity()};
    double left_cost{0.0};
    double right_cost{0.0};
    double up_cost{0.0};
    double down_cost{0.0};
    std::size_t considered_points{0};
    bool active{false};
  };

  void desired_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_desired_ = *msg;
    last_desired_time_ = this->now();
    has_desired_ = true;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_cloud_ = *msg;
    last_cloud_time_ = this->now();
    has_cloud_ = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void publish_avoided_cmd()
  {
    geometry_msgs::msg::Twist desired_cmd;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    nav_msgs::msg::Odometry odom_msg;
    bool has_desired = false;
    bool has_cloud = false;
    bool has_odom = false;
    rclcpp::Time desired_time(0, 0, this->get_clock()->get_clock_type());
    rclcpp::Time cloud_time(0, 0, this->get_clock()->get_clock_type());

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      desired_cmd = latest_desired_;
      cloud_msg = latest_cloud_;
      odom_msg = latest_odom_;
      has_desired = has_desired_;
      has_cloud = has_cloud_;
      has_odom = has_odom_;
      desired_time = last_desired_time_;
      cloud_time = last_cloud_time_;
    }

    const double input_timeout = this->get_parameter("input_timeout").as_double();
    const double cloud_timeout = this->get_parameter("cloud_timeout").as_double();
    const bool desired_stale = !has_desired || (this->now() - desired_time).seconds() > input_timeout;
    const bool cloud_stale = !has_cloud || (this->now() - cloud_time).seconds() > cloud_timeout;

    if (desired_stale) {
      desired_cmd = geometry_msgs::msg::Twist{};
    }

    geometry_msgs::msg::Twist final_cmd = desired_cmd;
    if (!cloud_stale && has_odom) {
      const auto summary = analyze_obstacles(cloud_msg, odom_msg);
      final_cmd = apply_avoidance(desired_cmd, summary);

      if (summary.active) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1500,
          "3D avoidance active: front=%.2f m, points=%zu",
          summary.front_clearance,
          summary.considered_points);
      }
    }

    limit_output(final_cmd);
    smooth_output(final_cmd);
    cmd_pub_->publish(previous_output_);
  }

  ObstacleSummary analyze_obstacles(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    const nav_msgs::msg::Odometry & odom_msg) const
  {
    ObstacleSummary summary;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(cloud_msg, cloud);
    if (cloud.empty()) {
      return summary;
    }

    const bool cloud_in_body_frame = this->get_parameter("cloud_in_body_frame").as_bool();
    const int max_points = this->get_parameter("max_points_to_process").as_int();
    const double forward_influence = this->get_parameter("forward_influence_distance").as_double();
    const double rear_influence = this->get_parameter("rear_influence_distance").as_double();
    const double side_influence = this->get_parameter("side_influence_distance").as_double();
    const double vertical_influence = this->get_parameter("vertical_influence_distance").as_double();
    const double safety_xy = this->get_parameter("safety_distance_xy").as_double();
    const double safety_z = this->get_parameter("safety_distance_z").as_double();

    tf2::Quaternion q(
      odom_msg.pose.pose.orientation.x,
      odom_msg.pose.pose.orientation.y,
      odom_msg.pose.pose.orientation.z,
      odom_msg.pose.pose.orientation.w);
    q.normalize();
    const tf2::Quaternion q_inv = q.inverse();
    const tf2::Vector3 vehicle_pos(
      odom_msg.pose.pose.position.x,
      odom_msg.pose.pose.position.y,
      odom_msg.pose.pose.position.z);

    const std::size_t stride = std::max<std::size_t>(
      1, cloud.size() / static_cast<std::size_t>(std::max(max_points, 1)));

    for (std::size_t i = 0; i < cloud.size(); i += stride) {
      const auto & point = cloud.points[i];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      tf2::Vector3 point_body;
      if (cloud_in_body_frame) {
        point_body = tf2::Vector3(point.x, point.y, point.z);
      } else {
        const tf2::Vector3 point_world(point.x, point.y, point.z);
        point_body = tf2::quatRotate(q_inv, point_world - vehicle_pos);
      }

      const double x = point_body.x();
      const double y = point_body.y();
      const double z = point_body.z();
      if (x > forward_influence || x < -rear_influence ||
        std::abs(y) > side_influence || std::abs(z) > vertical_influence)
      {
        continue;
      }

      const double nx = x >= 0.0 ? x / forward_influence : x / rear_influence;
      const double ny = y / side_influence;
      const double nz = z / vertical_influence;
      const double metric_distance = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (metric_distance > 1.0) {
        continue;
      }

      const double euclidean_distance = point_body.length();
      if (euclidean_distance < 1e-3) {
        continue;
      }

      const double influence = std::pow(1.0 - metric_distance, 2.0);
      summary.repulsion -= point_body.normalized() * (influence / std::max(euclidean_distance, 0.35));
      ++summary.considered_points;

      if (x > 0.0) {
        const double frontal_weight = influence * (1.0 + (forward_influence - x) / forward_influence);
        if (y >= 0.0) {
          summary.left_cost += frontal_weight;
        } else {
          summary.right_cost += frontal_weight;
        }
        if (z >= 0.0) {
          summary.up_cost += frontal_weight;
        } else {
          summary.down_cost += frontal_weight;
        }

        if (std::abs(y) < safety_xy && std::abs(z) < safety_z) {
          summary.front_clearance = std::min(summary.front_clearance, euclidean_distance);
        }
      }
    }

    summary.active =
      summary.considered_points > 0 &&
      (summary.front_clearance < forward_influence || summary.repulsion.length() > 1e-3);
    return summary;
  }

  geometry_msgs::msg::Twist apply_avoidance(
    const geometry_msgs::msg::Twist & desired_cmd,
    const ObstacleSummary & summary) const
  {
    geometry_msgs::msg::Twist output = desired_cmd;
    if (!summary.active) {
      return output;
    }

    const double repulsion_gain_xy = this->get_parameter("repulsion_gain_xy").as_double();
    const double repulsion_gain_z = this->get_parameter("repulsion_gain_z").as_double();
    const double lateral_escape_gain = this->get_parameter("lateral_escape_gain").as_double();
    const double vertical_escape_gain = this->get_parameter("vertical_escape_gain").as_double();
    const double safety_xy = this->get_parameter("safety_distance_xy").as_double();
    const double hard_stop_distance = this->get_parameter("hard_stop_distance").as_double();

    output.linear.x += repulsion_gain_xy * summary.repulsion.x();
    output.linear.y += repulsion_gain_xy * summary.repulsion.y();
    output.linear.z += repulsion_gain_z * summary.repulsion.z();

    const double left_right_total = summary.left_cost + summary.right_cost;
    if (left_right_total > 1e-3 && summary.front_clearance < safety_xy * 1.5) {
      const double side_bias =
        std::clamp((summary.right_cost - summary.left_cost) / left_right_total, -1.0, 1.0);
      output.linear.y += lateral_escape_gain * side_bias;
    }

    const double up_down_total = summary.up_cost + summary.down_cost;
    if (up_down_total > 1e-3 && summary.front_clearance < safety_xy * 1.5) {
      const double vertical_bias =
        std::clamp((summary.down_cost - summary.up_cost) / up_down_total, -1.0, 1.0);
      output.linear.z += vertical_escape_gain * vertical_bias;
    }

    if (desired_cmd.linear.x > 0.0 && summary.front_clearance < safety_xy) {
      const double scale = std::clamp(
        (summary.front_clearance - hard_stop_distance) /
        std::max(safety_xy - hard_stop_distance, 1e-3),
        0.0,
        1.0);
      output.linear.x = std::min(output.linear.x, desired_cmd.linear.x * scale);
    }

    return output;
  }

  void limit_output(geometry_msgs::msg::Twist & cmd) const
  {
    const double max_output_xy = this->get_parameter("max_output_xy").as_double();
    const double max_output_z = this->get_parameter("max_output_z").as_double();
    const double max_output_yaw_rate = this->get_parameter("max_output_yaw_rate").as_double();

    const double xy_speed = std::hypot(cmd.linear.x, cmd.linear.y);
    if (xy_speed > max_output_xy && xy_speed > 1e-6) {
      const double scale = max_output_xy / xy_speed;
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
    }

    cmd.linear.z = std::clamp(cmd.linear.z, -max_output_z, max_output_z);
    cmd.angular.z = std::clamp(cmd.angular.z, -max_output_yaw_rate, max_output_yaw_rate);
  }

  void smooth_output(const geometry_msgs::msg::Twist & target_cmd)
  {
    const double alpha = std::clamp(this->get_parameter("smoothing_alpha").as_double(), 0.0, 1.0);

    previous_output_.linear.x += alpha * (target_cmd.linear.x - previous_output_.linear.x);
    previous_output_.linear.y += alpha * (target_cmd.linear.y - previous_output_.linear.y);
    previous_output_.linear.z += alpha * (target_cmd.linear.z - previous_output_.linear.z);
    previous_output_.angular.z += alpha * (target_cmd.angular.z - previous_output_.angular.z);
  }

  std::mutex data_mutex_;
  geometry_msgs::msg::Twist latest_desired_;
  geometry_msgs::msg::Twist previous_output_;
  sensor_msgs::msg::PointCloud2 latest_cloud_;
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time last_desired_time_;
  rclcpp::Time last_cloud_time_;
  bool has_desired_{false};
  bool has_cloud_{false};
  bool has_odom_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Local3DAvoidance>());
  rclcpp::shutdown();
  return 0;
}
