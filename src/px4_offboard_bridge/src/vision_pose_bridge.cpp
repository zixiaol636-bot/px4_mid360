#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class VisionPoseBridge : public rclcpp::Node
{
public:
  VisionPoseBridge() : Node("vision_pose_bridge")
  {
    this->declare_parameter<std::string>("odom_topic", "/odom_filtered");
    this->declare_parameter<std::string>("vision_pose_topic", "/mavros/vision_pose/pose");
    this->declare_parameter<std::string>(
      "vision_pose_cov_topic", "/mavros/vision_pose/pose_cov");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("publish_rate", 30.0);

    const auto odom_topic = this->get_parameter("odom_topic").as_string();
    const auto pose_topic = this->get_parameter("vision_pose_topic").as_string();
    const auto pose_cov_topic = this->get_parameter("vision_pose_cov_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, rclcpp::QoS(10));
    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic, rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_latest_pose(); });

    RCLCPP_INFO(this->get_logger(), "VisionPoseBridge: %s -> %s", odom_topic.c_str(), pose_topic.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void publish_latest_pose()
  {
    nav_msgs::msg::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_) {
        return;
      }
      odom = latest_odom_;
    }

    const auto frame_id = this->get_parameter("frame_id").as_string();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose = odom.pose.pose;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = pose_msg.header;
    pose_cov_msg.pose = odom.pose;
    pose_cov_pub_->publish(pose_cov_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionPoseBridge>());
  rclcpp::shutdown();
  return 0;
}
