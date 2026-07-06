#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdometryRelay : public rclcpp::Node
{
public:
  OdometryRelay()
  : Node("odometry_relay"), first_msg_(true)
  {
    this->declare_parameter<std::string>("odom_topic_in", "/Odometry");
    this->declare_parameter<std::string>("odom_topic_out", "/odom_filtered");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_link_frame_id", "base_link");
    this->declare_parameter<double>("filter_alpha", 0.3);
    this->declare_parameter<bool>("publish_tf", true);

    const auto topic_in = this->get_parameter("odom_topic_in").as_string();
    const auto topic_out = this->get_parameter("odom_topic_out").as_string();

    raw_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_in, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      topic_out, rclcpp::QoS(10));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "OdometryRelay: %s -> %s", topic_in.c_str(), topic_out.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double alpha = this->get_parameter("filter_alpha").as_double();
    const auto odom_frame = this->get_parameter("odom_frame_id").as_string();
    const auto base_link_frame = this->get_parameter("base_link_frame_id").as_string();

    if (first_msg_) {
      filtered_odom_ = *msg;
      first_msg_ = false;
    } else {
      filtered_odom_.pose.pose.position.x =
        alpha * msg->pose.pose.position.x + (1.0 - alpha) * filtered_odom_.pose.pose.position.x;
      filtered_odom_.pose.pose.position.y =
        alpha * msg->pose.pose.position.y + (1.0 - alpha) * filtered_odom_.pose.pose.position.y;
      filtered_odom_.pose.pose.position.z =
        alpha * msg->pose.pose.position.z + (1.0 - alpha) * filtered_odom_.pose.pose.position.z;

      tf2::Quaternion q_prev;
      tf2::Quaternion q_new;
      tf2::fromMsg(filtered_odom_.pose.pose.orientation, q_prev);
      tf2::fromMsg(msg->pose.pose.orientation, q_new);
      const tf2::Quaternion q_interp = q_prev.slerp(q_new, alpha);
      filtered_odom_.pose.pose.orientation = tf2::toMsg(q_interp);

      filtered_odom_.twist.twist.linear.x =
        alpha * msg->twist.twist.linear.x + (1.0 - alpha) * filtered_odom_.twist.twist.linear.x;
      filtered_odom_.twist.twist.linear.y =
        alpha * msg->twist.twist.linear.y + (1.0 - alpha) * filtered_odom_.twist.twist.linear.y;
      filtered_odom_.twist.twist.linear.z =
        alpha * msg->twist.twist.linear.z + (1.0 - alpha) * filtered_odom_.twist.twist.linear.z;
      filtered_odom_.twist.twist.angular.x =
        alpha * msg->twist.twist.angular.x + (1.0 - alpha) * filtered_odom_.twist.twist.angular.x;
      filtered_odom_.twist.twist.angular.y =
        alpha * msg->twist.twist.angular.y + (1.0 - alpha) * filtered_odom_.twist.twist.angular.y;
      filtered_odom_.twist.twist.angular.z =
        alpha * msg->twist.twist.angular.z + (1.0 - alpha) * filtered_odom_.twist.twist.angular.z;
    }

    filtered_odom_.header = msg->header;
    filtered_odom_.header.frame_id = odom_frame;
    filtered_odom_.child_frame_id = base_link_frame;
    filtered_odom_.pose.covariance = msg->pose.covariance;
    filtered_odom_.twist.covariance = msg->twist.covariance;
    filtered_odom_pub_->publish(filtered_odom_);

    if (this->get_parameter("publish_tf").as_bool()) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = filtered_odom_.header;
      tf_msg.child_frame_id = base_link_frame;
      tf_msg.transform.translation.x = filtered_odom_.pose.pose.position.x;
      tf_msg.transform.translation.y = filtered_odom_.pose.pose.position.y;
      tf_msg.transform.translation.z = filtered_odom_.pose.pose.position.z;
      tf_msg.transform.rotation = filtered_odom_.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry filtered_odom_;
  bool first_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryRelay>());
  rclcpp::shutdown();
  return 0;
}
