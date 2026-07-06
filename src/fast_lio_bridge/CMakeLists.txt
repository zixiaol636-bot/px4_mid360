cmake_minimum_required(VERSION 3.8)
project(fast_lio_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(odometry_relay src/odometry_relay.cpp)
ament_target_dependencies(odometry_relay
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_executable(pointcloud_relay src/pointcloud_relay.cpp)
target_link_libraries(pointcloud_relay ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_relay
  pcl_conversions
  rclcpp
  sensor_msgs)

install(TARGETS
  odometry_relay
  pointcloud_relay
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
