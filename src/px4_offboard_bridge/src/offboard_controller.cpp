/**
 * @file offboard_controller.cpp
 * @brief 起飞/悬停/降落 状态机（演示版，默认不启动，避免和任务执行器抢控制）
 *
 * 状态机：
 *   IDLE → WAITING_OFFBOARD → ARMING → TAKEOFF → HOVER → LANDING → COMPLETE
 *    ↑                                                        │
 *    └────────────────── 自动循环 ←────────────────────────────┘（不实现，COMPLETE 即终点）
 *
 * ⚠️ 默认 auto_start=false，不会自动起飞。
 *   这个节点和 cmdvel_to_setpoint 用同一个 setpoint_velocity 话题，
 *   两者不能同时运行 → launch 里默认不启它，让 mission_executor 主导控制。
 *
 * 适用场景：单独测试起降（绑桨/系留），不跑完整任务。
 */

#include <algorithm>   // std::clamp
#include <chrono>      // std::chrono::duration / 200ms
#include <memory>      // std::make_shared
#include <string>      // std::string

#include <geometry_msgs/msg/pose_stamped.hpp>     // PX4 本地位置
#include <geometry_msgs/msg/twist_stamped.hpp>    // 速度指令（发给 MAVROS）
#include <mavros_msgs/msg/state.hpp>              // PX4 状态（连接/解锁/模式）
#include <mavros_msgs/srv/command_bool.hpp>        // 解锁/上锁服务
#include <mavros_msgs/srv/set_mode.hpp>            // 切换飞行模式服务
#include <rclcpp/rclcpp.hpp>                        // ROS2 核心

using namespace std::chrono_literals;  // 200ms 等时间字面量

class OffboardController : public rclcpp::Node
{
public:
  OffboardController()
  : Node("offboard_controller"),           // 节点名
    current_altitude_(0.0),                // 当前高度 [m]（NED: z 负=上升）
    home_altitude_(0.0),                   // 首次收到位置时记录的高度（起飞点基准）
    state_(State::IDLE),                   // 初始状态：空闲
    state_entered_at_(this->now())         // 进入当前状态的时间
  {
    // ===== 声明参数 =====
    this->declare_parameter<bool>("auto_start", false);              // ⚠️ false=不自动起飞，安全默认值
    this->declare_parameter<double>("setpoint_rate", 30.0);          // 控制循环频率 [Hz]
    this->declare_parameter<double>("takeoff_height", 2.0);          // 起飞目标高度 [m]
    this->declare_parameter<double>("takeoff_speed", 0.5);           // 起飞上升速度 [m/s]
    this->declare_parameter<double>("landing_speed", 0.3);           // 降落下降速度 [m/s]
    this->declare_parameter<double>("hover_duration", 0.0);          // 悬停时间 [秒]（0=不自动降落）
    this->declare_parameter<std::string>("offboard_mode", "OFFBOARD"); // PX4 模式名
    this->declare_parameter<std::string>("state_topic", "/mavros/state");
    this->declare_parameter<std::string>("local_position_topic", "/mavros/local_position/pose");
    this->declare_parameter<std::string>("setpoint_topic", "/mavros/setpoint_velocity/cmd_vel");
    this->declare_parameter<std::string>("arming_service", "/mavros/cmd/arming");
    this->declare_parameter<std::string>("set_mode_service", "/mavros/set_mode");

    // 读取参数
    const auto state_topic = this->get_parameter("state_topic").as_string();
    const auto local_position_topic = this->get_parameter("local_position_topic").as_string();
    const auto setpoint_topic = this->get_parameter("setpoint_topic").as_string();
    const double setpoint_rate = this->get_parameter("setpoint_rate").as_double();

    // ===== 订阅 PX4 状态（是否连接、解锁、当前模式）=====
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      state_topic, rclcpp::QoS(10),
      [this](const mavros_msgs::msg::State::SharedPtr msg) { vehicle_state_ = *msg; });

    // ===== 订阅 PX4 本地位置（取 z 高度）=====
    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      local_position_topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_altitude_ = msg->pose.position.z;  // NED: 负值=上升，正值=下降
        if (!home_altitude_initialized_) {
          home_altitude_ = current_altitude_;       // 首次收到的位置作为起飞点基准
          home_altitude_initialized_ = true;
        }
      });

    // ===== 发布速度指令（与 cmdvel_to_setpoint 共用同一话题！）=====
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      setpoint_topic, rclcpp::QoS(10));

    // ===== MAVROS 服务客户端 =====
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      this->get_parameter("arming_service").as_string());   // 解锁/上锁
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      this->get_parameter("set_mode_service").as_string());  // 切换飞行模式

    // ===== 控制定时器 =====
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / setpoint_rate),   // 周期
      [this]() { control_loop(); });
  }

private:
  // ===== 状态枚举 =====
  enum class State
  {
    IDLE,                // 空闲（等待 auto_start=true）
    WAITING_OFFBOARD,    // 正在发零速 setpoint + 等待 OFFBOARD 模式切换
    ARMING,              // 正在解锁
    TAKEOFF,             // 起飞中（用 P 控制器逼近目标高度）
    HOVER,               // 悬停（发零速，直到 hover_duration 到期）
    LANDING,             // 降落中（向 home_altitude_ 下降）
    COMPLETE              // 完成（已上锁，停在原地）
  };

  // ===== 状态机主循环 =====
  void control_loop()
  {
    // auto_start=false → 不发任何指令，只发零速（保持悬停）
    if (!this->get_parameter("auto_start").as_bool()) {
      publish_zero_velocity();
      return;
    }

    switch (state_) {
      case State::IDLE:
        transition_to(State::WAITING_OFFBOARD);    // 有 auto_start → 开始序列
        break;

      case State::WAITING_OFFBOARD:
        publish_zero_velocity();                    // ⚠️ 必须持续发 setpoint，PX4 才允许切 OFFBOARD
        if (vehicle_state_.mode == this->get_parameter("offboard_mode").as_string()) {
          transition_to(State::ARMING);             // 已经是 OFFBOARD → 走下一步
        } else if ((this->now() - state_entered_at_).seconds() > 1.0) {
          request_offboard_mode();                  // 重试切换（每秒一次）
          state_entered_at_ = this->now();          // 重置计时器
        }
        break;

      case State::ARMING:
        publish_zero_velocity();
        if (vehicle_state_.armed) {
          transition_to(State::TAKEOFF);            // 已解锁 → 开始起飞
        } else if ((this->now() - state_entered_at_).seconds() > 1.0) {
          request_arm(true);                        // 重试解锁
          state_entered_at_ = this->now();
        }
        break;

      case State::TAKEOFF:
        handle_takeoff();                           // 上升控制
        break;

      case State::HOVER:
        handle_hover();                             // 悬停控制
        break;

      case State::LANDING:
        handle_landing();                           // 下降控制
        break;

      case State::COMPLETE:
        publish_zero_velocity();                    // 保持零速（已在地面）
        break;
    }
  }

  // ===== 起飞：P 控制器 → 目标高度 = home_altitude_ + takeoff_height =====
  void handle_takeoff()
  {
    const double target_altitude = home_altitude_ + this->get_parameter("takeoff_height").as_double();
    const double error = target_altitude - current_altitude_;   // 误差 = 目标 - 当前

    // 到达容差内（15cm）→ 进入悬停
    if (std::abs(error) < 0.15) {
      transition_to(State::HOVER);
      return;
    }

    // 用 P 控制器发速度：vz = clamp(error, -takeoff_speed, takeoff_speed)
    // 误差越大速度越大，靠近目标时自动减速
    publish_vertical_velocity(std::clamp(
      error,
      -this->get_parameter("takeoff_speed").as_double(),   // 下限（负=上升，NED 中负 z=向上）
      this->get_parameter("takeoff_speed").as_double()));  // 上限
  }

  // ===== 悬停：零速 + 计时 =====
  void handle_hover()
  {
    publish_zero_velocity();                                // 零速 = 位置保持（PX4 内部控制器维持）
    const double hover_duration = this->get_parameter("hover_duration").as_double();
    if (hover_duration > 0.0 && (this->now() - state_entered_at_).seconds() >= hover_duration) {
      transition_to(State::LANDING);                       // 悬停时间到 → 降落
    }
  }

  // ===== 降落：P 控制器 → 目标 = home_altitude_ =====
  void handle_landing()
  {
    const double error = home_altitude_ - current_altitude_;  // 误差 = 地面 - 当前

    // 到达容差内（10cm）→ 上锁
    if (std::abs(error) < 0.1) {
      request_arm(false);                   // 上锁（disarm）
      transition_to(State::COMPLETE);
      return;
    }

    // P 控制器降速，clamp 在 landing_speed 范围内
    publish_vertical_velocity(std::clamp(
      error,
      -this->get_parameter("landing_speed").as_double(),
      this->get_parameter("landing_speed").as_double()));
  }

  // ===== 发布纯垂直速度 =====
  void publish_vertical_velocity(double vz)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";    // 机体坐标系（前/左/上）
    msg.twist.linear.z = vz;              // 只设 Z 轴，X/Y/角速度全为零
    setpoint_pub_->publish(msg);
  }

  // ===== 发布零速 =====
  void publish_zero_velocity()
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    // 不设置任何速度字段 → 默认全零
    setpoint_pub_->publish(msg);
  }

  // ===== 解锁/上锁服务 =====
  void request_arm(bool arm)
  {
    if (!arming_client_->wait_for_service(200ms)) {   // 等 200ms，服务不可用就跳过
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;                              // true=解锁，false=上锁
    arming_client_->async_send_request(request);       // 异步发送（不阻塞控制循环）
  }

  // ===== 切换 OFFBOARD 模式服务 =====
  void request_offboard_mode()
  {
    if (!set_mode_client_->wait_for_service(200ms)) {
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = this->get_parameter("offboard_mode").as_string();  // "OFFBOARD"
    set_mode_client_->async_send_request(request);
  }

  // ===== 状态切换（记录时间戳便于计算持续时间）=====
  void transition_to(State next_state)
  {
    state_ = next_state;
    state_entered_at_ = this->now();
  }

  // ---- 成员变量 ----
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;          // PX4 状态订阅
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_; // 位置订阅
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;   // 速度发布
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;        // 解锁服务
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;          // 模式服务
  rclcpp::TimerBase::SharedPtr timer_;           // 控制循环定时器

  mavros_msgs::msg::State vehicle_state_;        // 飞控当前状态（缓存）
  double current_altitude_;                      // 当前 NED 高度 (z 负=上升)
  double home_altitude_;                         // 起飞点基准高度
  bool home_altitude_initialized_{false};         // 是否已记录基准高度
  State state_;                                   // 当前状态
  rclcpp::Time state_entered_at_;                // 进入当前状态的时间戳
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
