cmake_minimum_required(VERSION 3.8)
project(px4_offboard_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(ego_planner_path_follower src/ego_planner_path_follower.cpp)
ament_target_dependencies(ego_planner_path_follower
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(local_3d_safety_filter src/local_3d_safety_filter.cpp)
target_link_libraries(local_3d_safety_filter ${PCL_LIBRARIES})
ament_target_dependencies(local_3d_safety_filter
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(px4_cmdvel_bridge src/px4_cmdvel_bridge.cpp)
ament_target_dependencies(px4_cmdvel_bridge
  geometry_msgs
  nav_msgs
  px4_msgs
  rclcpp
  std_srvs
  tf2
  tf2_geometry_msgs
)

add_executable(px4_visual_odometry_bridge src/px4_visual_odometry_bridge.cpp)
ament_target_dependencies(px4_visual_odometry_bridge
  nav_msgs
  px4_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(px4_status_bridge src/px4_status_bridge.cpp)
ament_target_dependencies(px4_status_bridge
  px4_msgs
  rclcpp
  rmw
  std_msgs
)

add_executable(ego_planner_ros2_adapter src/ego_planner_ros2_adapter.cpp)
ament_target_dependencies(ego_planner_ros2_adapter
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  ego_planner_path_follower
  local_3d_safety_filter
  px4_cmdvel_bridge
  px4_visual_odometry_bridge
  px4_status_bridge
  ego_planner_ros2_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
