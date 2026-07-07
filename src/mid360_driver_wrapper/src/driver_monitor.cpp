/**
 * @file driver_monitor.cpp
 * @brief 监控 MID360 驱动是否正常发布点云，断流超过阈值就告警
 *
 * 逻辑：订阅 /livox/lidar → 记录最新消息时间 → 定时检查间隔
 *       超过 watchdog_timeout → /driver/health 发 false
 *       恢复 → 发 true
 */

#include <chrono>    // std::chrono::duration 定时器周期
#include <memory>    // std::make_shared 智能指针
#include <string>    // std::string 参数名

#include <rclcpp/rclcpp.hpp>                      // ROS2 核心 Node / spin / shutdown
#include <sensor_msgs/msg/point_cloud2.hpp>        // MID360 点云消息类型
#include <std_msgs/msg/bool.hpp>                   // /driver/health 布尔消息

class DriverMonitor : public rclcpp::Node          // 继承 ROS2 Node 基类
{
public:
  DriverMonitor()                                   // 构造函数
  : Node("driver_monitor"),                         // 节点名
    last_msg_time_(this->now())                     // 初始化最后收包时间 = 当前时刻
  {
    // ---------- 声明可配置参数 ----------
    this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");  // 监听的点云话题
    this->declare_parameter<double>("watchdog_timeout", 2.0);             // 断流判定超时 [秒]
    this->declare_parameter<double>("check_rate", 10.0);                  // 看门狗检查频率 [Hz]
    this->declare_parameter<bool>("publish_health", true);                // 是否对外发布健康状态

    // ---------- 订阅 MID360 点云 ----------
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  // 创建订阅者
      this->get_parameter("lidar_topic").as_string(),  // topic 名（默认 /livox/lidar）
      rclcpp::SensorDataQoS(),                         // QoS：传感器数据（可靠、高带宽）
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {  // lambda 回调
        last_msg_time_ = this->now();                  // 只更新时间戳，不处理点云内容
      });

    // ---------- 可选发布健康状态 ----------
    if (this->get_parameter("publish_health").as_bool()) {       // 如果开启健康发布
      health_pub_ = this->create_publisher<std_msgs::msg::Bool>( // 创建发布者
        "/driver/health",                                        // 话题名
        rclcpp::QoS(1).transient_local());  // transient_local：晚订阅的节点也能拿到最新值
    }

    // ---------- 看门狗定时器 ----------
    const double check_rate = this->get_parameter("check_rate").as_double();  // 读取检查频率
    watchdog_timer_ = this->create_wall_timer(               // 创建系统时钟定时器（非 sim time）
      std::chrono::duration<double>(1.0 / check_rate),       // 周期 = 1/频率 秒，如 10Hz → 0.1s
      [this]() { watchdog_check(); });                       // 每次触发调用检查函数
  }

private:
  void watchdog_check()
  {
    if (!health_pub_) { return; }  // 未启用健康发布则跳过本次检查

    std_msgs::msg::Bool msg;                                                        // 创建消息
    msg.data =                                                                      // 填入健康状态
      (this->now() - last_msg_time_).seconds()                                      // 距上次收包的秒数
      <= this->get_parameter("watchdog_timeout").as_double();                       // 是否 <= 超时
    health_pub_->publish(msg);                                                      // 发布
  }

  // ---- 成员变量 ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;  // 点云订阅者
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;              // 健康状态发布者
  rclcpp::TimerBase::SharedPtr watchdog_timer_;                               // 定时器句柄
  rclcpp::Time last_msg_time_;  // 最后一次收到点云的 ROS 时间戳
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                             // 初始化 ROS2 客户端库
  rclcpp::spin(std::make_shared<DriverMonitor>());      // 创建节点并进入事件循环（阻塞）
  rclcpp::shutdown();                                   // 进程退出前清理 ROS2 资源
  return 0;
}
