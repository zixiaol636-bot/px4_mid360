#include <memory>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapLoader : public rclcpp::Node
{
public:
  MapLoader() : Node("map_loader")
  {
    this->declare_parameter<std::string>("map_file", "/tmp/map.pcd");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<bool>("load_on_start", false);

    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_cloud", rclcpp::QoS(1).transient_local().reliable());

    load_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/map/load",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = load_and_publish(this->get_parameter("map_file").as_string());
        response->message = response->success ? "Map loaded." : "Failed to load map.";
      });

    if (this->get_parameter("load_on_start").as_bool()) {
      load_and_publish(this->get_parameter("map_file").as_string());
    }
  }

private:
  bool load_and_publish(const std::string & filepath)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile(filepath, cloud) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD map: %s", filepath.c_str());
      return false;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("map_frame").as_string();
    map_pub_->publish(msg);
    last_map_ = msg;
    return true;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv_;
  sensor_msgs::msg::PointCloud2 last_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLoader>());
  rclcpp::shutdown();
  return 0;
}
