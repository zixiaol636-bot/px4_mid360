/**
 * @file cmdvel_to_setpoint.cpp
 * @brief 将 Nav2 /cmd_vel 平滑转换为 MAVROS 速度指令
 *
 * 核心功能：
 *   1. 接收 Nav2 DWB 输出的速度指令（FLU 坐标系）
 *   2. 做加速度限制（从 current 平滑逼近 target，避免阶跃）
 *   3. 做速度上限裁剪（clamp）
 *   4. 超时保护（Nav2 断流超过 cmd_vel_timeout → 速度归零，悬停）
 *   5. Z轴避障介入（obstacle_z_active_ 时用障碍检测的 vz 替代 Nav2 的 vz）
 *   6. 发布到 MAVROS 速度话题
 *
 * 数据流：
 *   /cmd_vel (Twist)  ──→  [超时检查] → [clamp] → [step_toward 平滑] → /mavros/setpoint_velocity/cmd_vel
 *   /obstacle/z_status  ──→  [Z轴覆盖] ──────────────────────────────────┘
 */

#include <algorithm>   // std::clamp, std::min, std::max
#include <chrono>      // std::chrono::duration 定时器
#include <memory>      // std::make_shared
#include <string>      // std::string 参数名

#include <geometry_msgs/msg/twist.hpp>           // 普通 Twist（输入）
#include <geometry_msgs/msg/twist_stamped.hpp>   // 带时间戳 TwistStamped（输出到 MAVROS）
#include <rclcpp/rclcpp.hpp>                      // ROS2 核心
#include <std_msgs/msg/float32.hpp>               // Z轴障碍速度值

class CmdVelToSetpoint : public rclcpp::Node     // 继承 ROS2 节点
{
public:
  CmdVelToSetpoint()
  : Node("cmdvel_to_setpoint"),                  // 节点名
    // ---- 初始化成员变量为 0 ----
    target_vx_(0.0),         // Nav2 发来的目标 x 速度
    target_vy_(0.0),         // Nav2 发来的目标 y 速度
    target_vz_(0.0),         // Nav2 发来的目标 z 速度
    target_yaw_rate_(0.0),  // Nav2 发来的目标偏航角速度
    current_vx_(0.0),        // 平滑后的当前 x 速度（实际发给 PX4 的值）
    current_vy_(0.0),        // 平滑后的当前 y 速度
    current_vz_(0.0),        // 平滑后的当前 z 速度
    current_yaw_rate_(0.0), // 平滑后的当前偏航角速度
    obstacle_z_override_(0.0), // Z轴避障覆盖值
    obstacle_z_active_(false), // Z轴避障是否激活
    last_cmd_vel_time_(this->now())  // 最后一次收到 /cmd_vel 的时间
  {
    // ===== 声明参数（可被 launch 文件覆盖）=====
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");                    // 输入：Nav2 速度指令
    this->declare_parameter<std::string>(
      "setpoint_topic", "/mavros/setpoint_velocity/cmd_vel");                             // 输出：MAVROS 速度话题
    this->declare_parameter<std::string>("obstacle_z_topic", "/obstacle/z_status");        // Z轴障碍检测结果
    this->declare_parameter<double>("publish_rate", 25.0);          // 发布频率 [Hz]
    this->declare_parameter<double>("acceleration_limit", 1.0);     // 加速度上限 [m/s²]
    this->declare_parameter<double>("max_vel_horizontal", 1.5);     // 水平最大速度 [m/s]
    this->declare_parameter<double>("max_vel_vertical", 0.5);       // 垂直最大速度 [m/s]
    this->declare_parameter<double>("max_yaw_rate", 1.0);           // 最大偏航角速度 [rad/s]
    this->declare_parameter<double>("cmd_vel_timeout", 0.5);        // /cmd_vel 断流超时 [秒]

    // 读取参数值（先从 declare 的默认值取，launch 传了则覆盖）
    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    const auto setpoint_topic = this->get_parameter("setpoint_topic").as_string();
    const auto obstacle_z_topic = this->get_parameter("obstacle_z_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    // ===== 订阅者1: /cmd_vel（Nav2 DWB 输出）=====
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10),     // topic 名 + 队列深度 10
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmd_vel_callback(msg); });

    // ===== 订阅者2: /obstacle/z_status（Z轴避障）=====
    obstacle_z_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      obstacle_z_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32::SharedPtr msg) { obstacle_callback(msg); });

    // ===== 发布者: 速度指令（发给 MAVROS → PX4）=====
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      setpoint_topic, rclcpp::QoS(10));

    // ===== 定时器: 固定频率调用平滑+发布 =====
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),  // 周期 = 1/频率 秒
      [this]() { publish_smoothed_setpoint(); });          // 每次触发执行
  }

private:
  // ===== 回调: Nav2 速度指令到达 =====
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_vx_ = msg->linear.x;         // 前向速度 (FLU x = 前)
    target_vy_ = msg->linear.y;         // 横向速度 (FLU y = 左)
    target_vz_ = msg->linear.z;         // 垂直速度 (FLU z = 上)
    target_yaw_rate_ = msg->angular.z;  // 偏航角速度
    last_cmd_vel_time_ = this->now();   // 更新最后收到时间（用于超时检测）
  }

  // ===== 回调: Z轴障碍检测结果 =====
  void obstacle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    obstacle_z_override_ = msg->data;                     // 存储覆盖值（正=上升，负=下降）
    obstacle_z_active_ = std::abs(msg->data) > 1e-4;      // 绝对值 > 0.0001 认为有障碍需要反应
  }

  // ===== 定时器回调: 平滑 + 限制 + 发布 =====
  void publish_smoothed_setpoint()
  {
    // 读取最新参数（支持运行时调参）
    const double publish_rate = this->get_parameter("publish_rate").as_double();
    const double accel_limit = this->get_parameter("acceleration_limit").as_double();
    const double max_vel_horizontal = this->get_parameter("max_vel_horizontal").as_double();
    const double max_vel_vertical = this->get_parameter("max_vel_vertical").as_double();
    const double max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
    const double cmd_vel_timeout = this->get_parameter("cmd_vel_timeout").as_double();
    const double dt = 1.0 / publish_rate;    // 每次平滑步进的时间间隔

    // ---- 超时检查 ----
    // 如果 Nav2 超过 cmd_vel_timeout 秒没发新指令 → 目标速度全归零 = 悬停
    const bool timed_out = (this->now() - last_cmd_vel_time_).seconds() > cmd_vel_timeout;

    double desired_vx = timed_out ? 0.0 : target_vx_;           // 超时 → 停下
    double desired_vy = timed_out ? 0.0 : target_vy_;
    double desired_vz = timed_out ? 0.0 : target_vz_;
    double desired_yaw_rate = timed_out ? 0.0 : target_yaw_rate_;

    // ---- Z轴避障覆盖 ----
    // obstacle_z_active_ = true 时，用障碍检测的速度替代 Nav2 的 vz（优先级更高）
    if (obstacle_z_active_) {
      desired_vz = obstacle_z_override_;
    }

    // ---- 速度上限钳制 (clamp) ----
    // 确保无论什么来源，最终速度都在安全范围内
    desired_vx = std::clamp(desired_vx, -max_vel_horizontal, max_vel_horizontal);
    desired_vy = std::clamp(desired_vy, -max_vel_horizontal, max_vel_horizontal);
    desired_vz = std::clamp(desired_vz, -max_vel_vertical, max_vel_vertical);
    desired_yaw_rate = std::clamp(desired_yaw_rate, -max_yaw_rate, max_yaw_rate);

    // ---- 加速度平滑 ----
    // step_toward: 从 current_ 向 desired_ 移动，但每步最多移 accel_limit * dt
    // 效果：速度变化变得平滑，不会从 0 突然跳到 1.5m/s
    current_vx_ = step_toward(current_vx_, desired_vx, accel_limit * dt);
    current_vy_ = step_toward(current_vy_, desired_vy, accel_limit * dt);
    current_vz_ = step_toward(current_vz_, desired_vz, accel_limit * dt);
    current_yaw_rate_ = step_toward(current_yaw_rate_, desired_yaw_rate, accel_limit * dt);

    // ---- 发布 ----
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();            // 时间戳
    msg.header.frame_id = "odom";   // Nav2 输出的是世界坐标系(odom)速度，设 odom 让 MAVROS 自动转成机体 FRD
    msg.twist.linear.x = current_vx_;           // 前向速度
    msg.twist.linear.y = current_vy_;           // 横向速度
    msg.twist.linear.z = current_vz_;           // 垂直速度
    msg.twist.angular.z = current_yaw_rate_;    // 偏航角速度
    setpoint_pub_->publish(msg);
  }

  // ===== 工具函数: 向目标值平滑步进 =====
  // current: 当前值  target: 目标值  max_delta: 单步最大变化量
  // 返回 current 向 target 靠近一"步"后的值
  static double step_toward(double current, double target, double max_delta)
  {
    if (target > current) {
      // 目标比当前大 → 增加，但不超 max_delta，也别超过目标
      return std::min(target, current + max_delta);
    }
    // 目标比当前小 → 减少
    return std::max(target, current - max_delta);
  }

  // ---- 成员变量 ----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;           // /cmd_vel 订阅
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_z_sub_;           // Z轴障碍订阅
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;       // 速度指令发布
  rclcpp::TimerBase::SharedPtr timer_;                                                 // 平滑发布定时器

  double target_vx_;        // Nav2 指令的 x 速度（每次 cmd_vel 到达时更新）
  double target_vy_;
  double target_vz_;
  double target_yaw_rate_;
  double current_vx_;       // 经平滑后实际发布的 x 速度（每周期递增逼近 target）
  double current_vy_;
  double current_vz_;
  double current_yaw_rate_;
  double obstacle_z_override_;   // Z轴避障覆盖速度
  bool obstacle_z_active_;       // 是否有 Z 轴障碍需要避让
  rclcpp::Time last_cmd_vel_time_;  // 最后收到 /cmd_vel 的时间戳
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                            // 初始化 ROS2
  rclcpp::spin(std::make_shared<CmdVelToSetpoint>());  // 创建节点并进入事件循环
  rclcpp::shutdown();                                   // 清理
  return 0;
}
