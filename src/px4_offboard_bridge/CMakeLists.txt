cmake_minimum_required(VERSION 3.8)
project(px4_offboard_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(vision_pose_bridge src/vision_pose_bridge.cpp)
ament_target_dependencies(vision_pose_bridge
  geometry_msgs
  nav_msgs
  rclcpp)

add_executable(offboard_controller src/offboard_controller.cpp)
ament_target_dependencies(offboard_controller
  geometry_msgs
  mavros_msgs
  rclcpp)

add_executable(cmdvel_to_setpoint src/cmdvel_to_setpoint.cpp)
ament_target_dependencies(cmdvel_to_setpoint
  geometry_msgs
  rclcpp
  std_msgs)

add_executable(local_3d_avoidance src/local_3d_avoidance.cpp)
target_link_libraries(local_3d_avoidance ${PCL_LIBRARIES})
ament_target_dependencies(local_3d_avoidance
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs)

add_executable(z_axis_monitor src/z_axis_monitor.cpp)
target_link_libraries(z_axis_monitor ${PCL_LIBRARIES})
ament_target_dependencies(z_axis_monitor
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs)

install(TARGETS
  vision_pose_bridge
  offboard_controller
  cmdvel_to_setpoint
  local_3d_avoidance
  z_axis_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
