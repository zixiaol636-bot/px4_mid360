#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class CmdVelToSetpoint : public rclcpp::Node
{
public:
  CmdVelToSetpoint()
  : Node("cmdvel_to_setpoint"),
    target_vx_(0.0),
    target_vy_(0.0),
    target_vz_(0.0),
    target_yaw_rate_(0.0),
    current_vx_(0.0),
    current_vy_(0.0),
    current_vz_(0.0),
    current_yaw_rate_(0.0),
    obstacle_z_override_(0.0),
    obstacle_z_active_(false),
    last_cmd_vel_time_(this->now())
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>(
      "setpoint_topic", "/mavros/setpoint_velocity/cmd_vel");
    this->declare_parameter<std::string>("obstacle_z_topic", "/obstacle/z_status");
    this->declare_parameter<double>("publish_rate", 25.0);
    this->declare_parameter<double>("acceleration_limit", 1.0);
    this->declare_parameter<double>("max_vel_horizontal", 1.5);
    this->declare_parameter<double>("max_vel_vertical", 0.5);
    this->declare_parameter<double>("max_yaw_rate", 1.0);
    this->declare_parameter<double>("cmd_vel_timeout", 0.5);

    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    const auto setpoint_topic = this->get_parameter("setpoint_topic").as_string();
    const auto obstacle_z_topic = this->get_parameter("obstacle_z_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_callback(msg); });

    obstacle_z_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      obstacle_z_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::SharedPtr msg) { obstacle_callback(msg); });

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      setpoint_topic, rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_smoothed_setpoint(); });
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_vx_ = msg->linear.x;
    target_vy_ = msg->linear.y;
    target_vz_ = msg->linear.z;
    target_yaw_rate_ = msg->angular.z;
    last_cmd_vel_time_ = this->now();
  }

  void obstacle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    obstacle_z_override_ = msg->data;
    obstacle_z_active_ = std::abs(msg->data) > 1e-4;
  }

  void publish_smoothed_setpoint()
  {
    const double publish_rate = this->get_parameter("publish_rate").as_double();
    const double accel_limit = this->get_parameter("acceleration_limit").as_double();
    const double max_vel_horizontal = this->get_parameter("max_vel_horizontal").as_double();
    const double max_vel_vertical = this->get_parameter("max_vel_vertical").as_double();
    const double max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
    const double cmd_vel_timeout = this->get_parameter("cmd_vel_timeout").as_double();
    const double dt = 1.0 / publish_rate;

    const bool timed_out = (this->now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout;

    double desired_vx = timed_out ? 0.0 : target_vx_;
    double desired_vy = timed_out ? 0.0 : target_vy_;
    double desired_vz = timed_out ? 0.0 : target_vz_;
    double desired_yaw_rate = timed_out ? 0.0 : target_yaw_rate_;

    if (obstacle_z_active_) {
      desired_vz = obstacle_z_override_;
    }

    desired_vx = std::clamp(desired_vx, -max_vel_horizontal, max_vel_horizontal);
    desired_vy = std::clamp(desired_vy, -max_vel_horizontal, max_vel_horizontal);
    desired_vz = std::clamp(desired_vz, -max_vel_vertical, max_vel_vertical);
    desired_yaw_rate = std::clamp(desired_yaw_rate, -max_yaw_rate, max_yaw_rate);

    current_vx_ = step_toward(current_vx_, desired_vx, accel_limit * dt);
    current_vy_ = step_toward(current_vy_, desired_vy, accel_limit * dt);
    current_vz_ = step_toward(current_vz_, desired_vz, accel_limit * dt);
    current_yaw_rate_ = step_toward(current_yaw_rate_, desired_yaw_rate, accel_limit * dt);

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = current_vx_;
    msg.twist.linear.y = current_vy_;
    msg.twist.linear.z = current_vz_;
    msg.twist.angular.z = current_yaw_rate_;
    setpoint_pub_->publish(msg);
  }

  static double step_toward(double current, double target, double max_delta)
  {
    if (target > current) {
      return std::min(target, current + max_delta);
    }
    return std::max(target, current - max_delta);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_z_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double target_vx_;
  double target_vy_;
  double target_vz_;
  double target_yaw_rate_;
  double current_vx_;
  double current_vy_;
  double current_vz_;
  double current_yaw_rate_;
  double obstacle_z_override_;
  bool obstacle_z_active_;
  rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToSetpoint>());
  rclcpp::shutdown();
  return 0;
}
