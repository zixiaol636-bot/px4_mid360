#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class CommunicationWatchdog : public rclcpp::Node
{
public:
  CommunicationWatchdog()
  : Node("communication_watchdog"), last_msg_time_(this->now())
  {
    this->declare_parameter<std::string>("heartbeat_topic", "/mavros/local_position/pose");
    this->declare_parameter<double>("heartbeat_timeout", 1.0);
    this->declare_parameter<double>("failsafe_timeout", 5.0);
    this->declare_parameter<double>("watchdog_rate", 10.0);

    heartbeat_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("heartbeat_topic").as_string(), rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr) { last_msg_time_ = this->now(); });

    comms_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/safety/comms_ok", rclcpp::QoS(1).transient_local());
    failsafe_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/safety/failsafe", rclcpp::QoS(1).transient_local());

    const double rate = this->get_parameter("watchdog_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      [this]() { evaluate_link(); });
  }

private:
  void evaluate_link()
  {
    const double elapsed = (this->now() - last_msg_time_).seconds();
    const bool comms_ok = elapsed <= this->get_parameter("heartbeat_timeout").as_double();
    const bool failsafe = elapsed > this->get_parameter("failsafe_timeout").as_double();

    if (comms_ok != last_comms_ok_) {
      std_msgs::msg::Bool msg;
      msg.data = comms_ok;
      comms_pub_->publish(msg);
      last_comms_ok_ = comms_ok;
    }

    if (failsafe != last_failsafe_) {
      std_msgs::msg::Bool msg;
      msg.data = failsafe;
      failsafe_pub_->publish(msg);
      last_failsafe_ = failsafe;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr comms_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr failsafe_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_msg_time_;
  bool last_comms_ok_{false};
  bool last_failsafe_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommunicationWatchdog>());
  rclcpp::shutdown();
  return 0;
}
