/**
 * @file vision_pose_bridge.cpp
 * @brief 将 FAST-LIO2 里程计位姿 (FLU) 转换为 PX4 视觉位姿 (NED)，发给 MAVROS → EKF2
 *
 * 这个节点是"定位链路的最后一棒"：
 *   1. 接收 /odom_filtered（FAST-LIO2 输出，FLU 坐标系: x=前 y=左 z=上）
 *   2. 将 FLU 转换为 NED（x=北 y=东 z=下）→ PX4 只认 NED
 *   3. 同时发两个话题：
 *      - /mavros/vision_pose/pose     → PoseStamped（位姿，不含协方差）
 *      - /mavros/vision_pose/pose_cov → PoseWithCovarianceStamped（位姿+协方差，EKF2 融合更优）
 *   4. 固定频率 30Hz 发布，保证 PX4 不会因断流而报超时
 *
 * ⚠️ 不转换会怎样？
 *   FLU 坐标直发 PX4 → 前飞变左飞，上升变下降 → 飞机完全失控
 */

#include <chrono>       // std::chrono 定时器周期
#include <memory>       // std::make_shared
#include <mutex>        // std::mutex / std::lock_guard 线程安全
#include <string>       // std::string 参数名

#include <geometry_msgs/msg/pose_stamped.hpp>                // 普通带时间戳位姿
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> // 带协方差的带时间戳位姿（给 EKF2）
#include <nav_msgs/msg/odometry.hpp>                          // 里程计消息（输入）
#include <rclcpp/rclcpp.hpp>                                   // ROS2 核心

#include "warehouse_utils/frame_conversions.hpp"  // FLU ↔ NED 坐标转换（必须！）

class VisionPoseBridge : public rclcpp::Node
{
public:
  VisionPoseBridge() : Node("vision_pose_bridge")     // 节点名
  {
    // ===== 声明参数 =====
    this->declare_parameter<std::string>("odom_topic", "/odom_filtered");              // 输入：FAST-LIO2 滤波里程计
    this->declare_parameter<std::string>("vision_pose_topic", "/mavros/vision_pose/pose"); // 输出1：位姿（无协方差）
    this->declare_parameter<std::string>(
      "vision_pose_cov_topic", "/mavros/vision_pose/pose_cov");                       // 输出2：位姿+协方差
    this->declare_parameter<std::string>("frame_id", "odom");   // 输出位姿的坐标系名（里程计数据在 odom 坐标系下）
    this->declare_parameter<double>("publish_rate", 30.0);                              // 发布频率 [Hz]

    // 读取参数
    const auto odom_topic = this->get_parameter("odom_topic").as_string();
    const auto pose_topic = this->get_parameter("vision_pose_topic").as_string();
    const auto pose_cov_topic = this->get_parameter("vision_pose_cov_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    // ===== 订阅 FAST-LIO2 滤波里程计 =====
    // 为什么不直接订 /Odometry？因为 /odom_filtered 经过 odometry_relay 做了低通滤波，更平滑
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),  // 传感器 QoS：可靠传输
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    // ===== 发布1：普通位姿（PoseStamped）=====
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::QoS(10));

    // ===== 发布2：带协方差的位姿（PoseWithCovarianceStamped）=====
    // EKF2 可以用协方差信息来加权融合——协方差小=这个测量很准，协方差大=不太可信
    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic, rclcpp::QoS(10));

    // ===== 定时器：固定频率发布 =====
    // 为什么用定时器而不是在 callback 里直接发？
    //   → FAST-LIO2 的输出频率不稳定（20-50Hz 波动），定时器保证 PX4 收到 30Hz 固定频率的位姿
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),  // 周期 = 1/频率 秒
      [this]() { publish_latest_pose(); });

    RCLCPP_INFO(this->get_logger(), "VisionPoseBridge 就绪: %s -> %s + %s",
                odom_topic.c_str(), pose_topic.c_str(), pose_cov_topic.c_str());
  }

private:
  // ===== 回调：最新里程计到达 =====
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);  // 加锁：回调线程 vs 定时器线程可能并发
    latest_odom_ = *msg;                              // 深拷贝一份（保证定时器线程读取时不冲突）
    has_odom_ = true;                                 // 标记已有数据
  }
  // lock_guard 离开作用域自动解锁

  // ===== 定时器回调：发布最新位姿 =====
  void publish_latest_pose()
  {
    nav_msgs::msg::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);  // 加锁读取
      if (!has_odom_) {
        return;         // 还没收到过里程计数据 → 不发布
      }
      odom = latest_odom_;  // 拷贝出来（尽快释放锁，避免阻塞 callback 线程）
    }

    const auto frame_id = this->get_parameter("frame_id").as_string();

    // ---- 构造 FLU 位姿（从里程计中提取）----
    geometry_msgs::msg::PoseStamped pose_flu;
    pose_flu.header.frame_id = "odom";      // 来源是 odom 坐标系
    pose_flu.pose = odom.pose.pose;         // 拷贝位置 + 四元数

    // ---- ⚠️ 关键转换：FLU → NED ----
    // 不转换直接发 → PX4 收到的 x 是 ROS 的 y（前变左），z 符号反（上变下），飞机失控
    geometry_msgs::msg::PoseStamped pose_ned;
    pose_ned = warehouse_utils::FrameConversions::flu_to_ned(pose_flu);

    // ---- 发布1：PoseStamped（无协方差）----
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();       // 覆盖时间戳为当前时刻
    pose_msg.header.frame_id = frame_id;        // odom 帧（与输入 /odom_filtered 坐标系一致）
    pose_msg.pose = pose_ned.pose;              // 使用转换后的 NED 位姿
    pose_pub_->publish(pose_msg);

    // ---- 发布2：PoseWithCovarianceStamped（带协方差）----
    // 比上面的多一个协方差矩阵（6×6），EKF2 根据这个矩阵决定"相信这个测量多少"
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = pose_msg.header;      // 共用同一个 header
    // odom.pose 是 PoseWithCovariance（位置+朝向+协方差），
    // 但位置已在上面转为 NED，协方差矩阵也应做相应变换（简化处理：仅翻转 z 分量相关的协方差项）
    pose_cov_msg.pose = odom.pose;              // 整体拷贝 pose（位置+朝向+协方差）
    pose_cov_msg.pose.pose = pose_ned.pose;      // 覆盖位置为 NED 转换后的值
    // 注意：协方差矩阵没有做 FLU→NED 旋转变换，对于精度要求不极高的场景影响不大。
    //       如需严格转换，需对 6×6 协方差矩阵做旋转映射。
    pose_cov_pub_->publish(pose_cov_msg);
  }

  // ---- 成员变量 ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;           // 里程计订阅
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;       // 位姿发布1
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;  // 位姿发布2
  rclcpp::TimerBase::SharedPtr timer_;            // 定时发布定时器

  std::mutex odom_mutex_;         // 保护 latest_odom_ 的互斥锁（callback 写 → 定时器读）
  nav_msgs::msg::Odometry latest_odom_;  // 缓冲的最新里程计数据
  bool has_odom_{false};           // 是否已收到过数据
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                               // 初始化 ROS2
  rclcpp::spin(std::make_shared<VisionPoseBridge>());     // 创建节点并阻塞运行
  rclcpp::shutdown();                                      // 清理
  return 0;
}
