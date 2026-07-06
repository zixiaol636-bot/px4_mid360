#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class OffboardController : public rclcpp::Node
{
public:
  OffboardController()
  : Node("offboard_controller"),
    current_altitude_(0.0),
    home_altitude_(0.0),
    state_(State::IDLE),
    state_entered_at_(this->now())
  {
    this->declare_parameter<bool>("auto_start", false);
    this->declare_parameter<double>("setpoint_rate", 30.0);
    this->declare_parameter<double>("takeoff_height", 2.0);
    this->declare_parameter<double>("takeoff_speed", 0.5);
    this->declare_parameter<double>("landing_speed", 0.3);
    this->declare_parameter<double>("hover_duration", 0.0);
    this->declare_parameter<std::string>("offboard_mode", "OFFBOARD");
    this->declare_parameter<std::string>("state_topic", "/mavros/state");
    this->declare_parameter<std::string>("local_position_topic", "/mavros/local_position/pose");
    this->declare_parameter<std::string>(
      "setpoint_topic", "/mavros/setpoint_velocity/cmd_vel");
    this->declare_parameter<std::string>("arming_service", "/mavros/cmd/arming");
    this->declare_parameter<std::string>("set_mode_service", "/mavros/set_mode");

    const auto state_topic = this->get_parameter("state_topic").as_string();
    const auto local_position_topic = this->get_parameter("local_position_topic").as_string();
    const auto setpoint_topic = this->get_parameter("setpoint_topic").as_string();
    const double setpoint_rate = this->get_parameter("setpoint_rate").as_double();

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      state_topic, rclcpp::QoS(10),
      [this](const mavros_msgs::msg::State::SharedPtr msg) { vehicle_state_ = *msg; });

    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      local_position_topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_altitude_ = msg->pose.position.z;
        if (!home_altitude_initialized_) {
          home_altitude_ = current_altitude_;
          home_altitude_initialized_ = true;
        }
      });

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      setpoint_topic, rclcpp::QoS(10));

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      this->get_parameter("arming_service").as_string());
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      this->get_parameter("set_mode_service").as_string());

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / setpoint_rate),
      [this]() { control_loop(); });
  }

private:
  enum class State
  {
    IDLE,
    WAITING_OFFBOARD,
    ARMING,
    TAKEOFF,
    HOVER,
    LANDING,
    COMPLETE
  };

  void control_loop()
  {
    if (!this->get_parameter("auto_start").as_bool()) {
      publish_zero_velocity();
      return;
    }

    switch (state_) {
      case State::IDLE:
        transition_to(State::WAITING_OFFBOARD);
        break;
      case State::WAITING_OFFBOARD:
        publish_zero_velocity();
        if (vehicle_state_.mode == this->get_parameter("offboard_mode").as_string()) {
          transition_to(State::ARMING);
        } else if ((this->now() - state_entered_at_).seconds() > 1.0) {
          request_offboard_mode();
          state_entered_at_ = this->now();
        }
        break;
      case State::ARMING:
        publish_zero_velocity();
        if (vehicle_state_.armed) {
          transition_to(State::TAKEOFF);
        } else if ((this->now() - state_entered_at_).seconds() > 1.0) {
          request_arm(true);
          state_entered_at_ = this->now();
        }
        break;
      case State::TAKEOFF:
        handle_takeoff();
        break;
      case State::HOVER:
        handle_hover();
        break;
      case State::LANDING:
        handle_landing();
        break;
      case State::COMPLETE:
        publish_zero_velocity();
        break;
    }
  }

  void handle_takeoff()
  {
    const double target_altitude = home_altitude_ + this->get_parameter("takeoff_height").as_double();
    const double error = target_altitude - current_altitude_;

    if (std::abs(error) < 0.15) {
      transition_to(State::HOVER);
      return;
    }

    publish_vertical_velocity(std::clamp(
      error,
      -this->get_parameter("takeoff_speed").as_double(),
      this->get_parameter("takeoff_speed").as_double()));
  }

  void handle_hover()
  {
    publish_zero_velocity();
    const double hover_duration = this->get_parameter("hover_duration").as_double();
    if (hover_duration > 0.0 && (this->now() - state_entered_at_).seconds() >= hover_duration) {
      transition_to(State::LANDING);
    }
  }

  void handle_landing()
  {
    const double error = home_altitude_ - current_altitude_;
    if (std::abs(error) < 0.1) {
      request_arm(false);
      transition_to(State::COMPLETE);
      return;
    }

    publish_vertical_velocity(std::clamp(
      error,
      -this->get_parameter("landing_speed").as_double(),
      this->get_parameter("landing_speed").as_double()));
  }

  void publish_vertical_velocity(double vz)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    msg.twist.linear.z = vz;
    setpoint_pub_->publish(msg);
  }

  void publish_zero_velocity()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    setpoint_pub_->publish(msg);
  }

  void request_arm(bool arm)
  {
    if (!arming_client_->wait_for_service(200ms)) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    arming_client_->async_send_request(request);
  }

  void request_offboard_mode()
  {
    if (!set_mode_client_->wait_for_service(200ms)) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = this->get_parameter("offboard_mode").as_string();
    set_mode_client_->async_send_request(request);
  }

  void transition_to(State next_state)
  {
    state_ = next_state;
    state_entered_at_ = this->now();
  }

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::State vehicle_state_;
  double current_altitude_;
  double home_altitude_;
  bool home_altitude_initialized_{false};
  State state_;
  rclcpp::Time state_entered_at_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
