#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class EkfHealthMonitor : public rclcpp::Node
{
public:
  EkfHealthMonitor() : Node("ekf_health_monitor")
  {
    this->declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom");
    this->declare_parameter<std::string>("status_topic", "/safety/ekf_health");
    this->declare_parameter<double>("max_position_covariance", 0.5);
    this->declare_parameter<double>("max_orientation_covariance", 0.1);
    this->declare_parameter<double>("max_velocity_covariance", 1.0);
    this->declare_parameter<int>("consecutive_failures_threshold", 5);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("status_topic").as_string(), rclcpp::QoS(1).transient_local());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double max_position_covariance =
      this->get_parameter("max_position_covariance").as_double();
    const double max_orientation_covariance =
      this->get_parameter("max_orientation_covariance").as_double();
    const double max_velocity_covariance =
      this->get_parameter("max_velocity_covariance").as_double();
    const int threshold =
      this->get_parameter("consecutive_failures_threshold").as_int();

    const bool healthy =
      msg->pose.covariance[0] <= max_position_covariance &&
      msg->pose.covariance[7] <= max_position_covariance &&
      msg->pose.covariance[14] <= max_position_covariance &&
      msg->pose.covariance[21] <= max_orientation_covariance &&
      msg->pose.covariance[28] <= max_orientation_covariance &&
      msg->pose.covariance[35] <= max_orientation_covariance &&
      msg->twist.covariance[0] <= max_velocity_covariance &&
      msg->twist.covariance[7] <= max_velocity_covariance &&
      msg->twist.covariance[14] <= max_velocity_covariance;

    consecutive_failures_ = healthy ? 0 : consecutive_failures_ + 1;
    const bool filtered_healthy = consecutive_failures_ < threshold;

    if (filtered_healthy != last_published_healthy_) {
      std_msgs::msg::Bool status;
      status.data = filtered_healthy;
      status_pub_->publish(status);
      last_published_healthy_ = filtered_healthy;
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  int consecutive_failures_{0};
  bool last_published_healthy_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
