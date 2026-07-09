cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)

add_executable(geofence_monitor src/geofence_monitor.cpp)
ament_target_dependencies(geofence_monitor
  nav_msgs
  rclcpp
  std_msgs)

add_executable(battery_monitor src/battery_monitor.cpp)
ament_target_dependencies(battery_monitor
  px4_msgs
  rclcpp
  rmw
  std_msgs)

add_executable(ekf_health_monitor src/ekf_health_monitor.cpp)
ament_target_dependencies(ekf_health_monitor
  nav_msgs
  rclcpp
  std_msgs)

add_executable(communication_watchdog src/communication_watchdog.cpp)
ament_target_dependencies(communication_watchdog
  rclcpp
  std_msgs)

install(TARGETS
  geofence_monitor
  battery_monitor
  ekf_health_monitor
  communication_watchdog
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
