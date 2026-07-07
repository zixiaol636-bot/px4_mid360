/**
 * @file z_axis_monitor.cpp
 * @brief Z轴悬空障碍检测 — 补充 Nav2 DWB 的 2D 盲区
 *
 * 问题：Nav2 DWB 只看 2D 代价地图（XY 平面），看不见头顶的横梁、吊管。
 *       无人机保持飞行高度直线飞行时会直接撞上去。
 *
 * 解决：本节点检测飞行高度 ±2m 范围内的点云密度
 *       → 头顶有障碍 → 发下降指令
 *       → 脚下有障碍 → 发上升指令
 *       → cmdvel_to_setpoint 收到后覆盖 Nav2 的 vz
 *
 * 数据流：
 *   /cloud_registered (PointCloud2)  →  [PCL 裁剪 ±2m]  →  [计点密度]
 *     →  [算平均 Z]  →  [判断上/下]  →  /obstacle/z_status (Float32)
 */

#include <algorithm>   // std::clamp
#include <chrono>      // 定时器
#include <memory>      // std::make_shared
#include <string>      // std::string

#include <pcl/filters/passthrough.h>     // PCL 直通滤波（按 Z 轴裁剪）
#include <pcl/point_cloud.h>              // PCL 点云容器
#include <pcl/point_types.h>              // pcl::PointXYZI 类型
#include <pcl_conversions/pcl_conversions.h>  // ROS 消息 ↔ PCL 互转
#include <rclcpp/rclcpp.hpp>               // ROS2 核心
#include <sensor_msgs/msg/point_cloud2.hpp> // 点云消息
#include <std_msgs/msg/float32.hpp>         // Z轴障碍速度值

class ZAxisMonitor : public rclcpp::Node
{
public:
  ZAxisMonitor() : Node("z_axis_monitor")     // 节点名
  {
    // ===== 声明参数 =====
    this->declare_parameter<std::string>("pointcloud_topic", "/cloud_registered");  // 输入：FAST-LIO2 实时点云
    this->declare_parameter<std::string>("obstacle_z_topic", "/obstacle/z_status");  // 输出：Z轴避障速度
    this->declare_parameter<std::vector<double>>("check_range", {-2.0, 2.0});  // 检测范围 [下, 上] 米，相对于 LiDAR
    this->declare_parameter<int>("point_density_threshold", 15);     // 体素内最少点数，低于此值不算障碍
    this->declare_parameter<double>("min_z_response", -0.3);         // 头顶有障碍 → 降速 [m/s] (负值=下降)
    this->declare_parameter<double>("max_z_response", 0.3);          // 脚下有障碍 → 升速 [m/s] (正值=上升)
    this->declare_parameter<double>("clear_timeout", 1.0);           // 障碍消失后保持响应 [秒]
    this->declare_parameter<double>("publish_rate", 5.0);            // 检测频率 [Hz]

    const auto pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    const auto obstacle_z_topic = this->get_parameter("obstacle_z_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    // ===== 订阅 FAST-LIO2 实时点云 =====
    // 只缓冲最新一帧，不逐帧处理（每周期取 latest_cloud_）
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),  // 传感器 QoS：高带宽、可靠
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_cloud_ = msg; });

    // ===== 发布 Z 轴速度覆盖值 =====
    obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      obstacle_z_topic, rclcpp::QoS(10));

    // ===== 定时器：固定频率执行检测 =====
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),  // 周期 = 1/频率
      [this]() { evaluate_latest_cloud(); });
  }

private:
  // ===== 核心函数：检测最新点云是否有 Z 轴障碍 =====
  void evaluate_latest_cloud()
  {
    // 还没收到过点云 → 发 0（无障碍）
    if (!latest_cloud_) {
      publish_override(0.0);
      return;
    }

    // ROS 点云 → PCL 点云（PointXYZI = 含 x/y/z/intensity）
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*latest_cloud_, cloud);
    if (cloud.empty()) {
      publish_override(0.0);     // 空点云 → 无障碍
      return;
    }

    // 读取参数
    const auto check_range = this->get_parameter("check_range").as_double_array();  // [z_min, z_max]
    const int density_threshold = this->get_parameter("point_density_threshold").as_int();
    const double min_z_response = this->get_parameter("min_z_response").as_double();
    const double max_z_response = this->get_parameter("max_z_response").as_double();

    // ---- PCL 直通滤波：只保留 Z 在 [check_range[0], check_range[1]] 之间的点 ----
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud.makeShared());              // 输入点云
    pass.setFilterFieldName("z");                         // 按 Z 轴过滤
    pass.setFilterLimits(check_range[0], check_range[1]); // 裁剪范围

    pcl::PointCloud<pcl::PointXYZI> filtered;
    pass.filter(filtered);                               // 执行过滤

    // 裁剪后的点数太少 → 无障碍（噪声级别）
    if (static_cast<int>(filtered.size()) < density_threshold) {
      publish_override(0.0);
      return;
    }

    // ---- 计算裁剪区域内点云的平均 Z 值 ----
    double average_z = 0.0;
    for (const auto & point : filtered.points) {
      average_z += point.z;     // 累加所有点的 z
    }
    average_z /= static_cast<double>(filtered.size());  // 平均

    // ---- 判断障碍方向 + 选择响应 ----
    // ROS FLU 约定: z 正 = 上
    //   average_z > 0  →  障碍在 LiDAR 上方（头顶横梁）→  需要下降 →  min_z_response（负值）
    //   average_z < 0  →  障碍在 LiDAR 下方（地面堆货）→  需要上升 →  max_z_response（正值）
    const double z_override = average_z > 0.0 ? min_z_response : max_z_response;
    publish_override(z_override);
  }

  // ===== 发布 Z 轴速度 =====
  void publish_override(double value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);           // double → float（消息字段是 float32）
    obstacle_pub_->publish(msg);
  }

  // ---- 成员变量 ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;  // 点云订阅
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_pub_;         // Z轴速度发布
  rclcpp::TimerBase::SharedPtr timer_;            // 检测定时器
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;  // 缓冲最新一帧点云
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZAxisMonitor>());
  rclcpp::shutdown();
  return 0;
}
