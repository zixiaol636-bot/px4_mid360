cmake_minimum_required(VERSION 3.8)
project(waypoint_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(warehouse_utils REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/WaypointArray.msg"
  "srv/SaveWaypoints.srv"
  "srv/LoadWaypoints.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(waypoint_manager src/waypoint_manager.cpp)
ament_target_dependencies(waypoint_manager
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  visualization_msgs
  warehouse_utils
)
target_link_libraries(waypoint_manager
  yaml_utils
  "${cpp_typesupport_target}"
)

add_executable(mission_executor src/mission_executor.cpp)
ament_target_dependencies(mission_executor
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  warehouse_utils
)
target_link_libraries(mission_executor
  yaml_utils
  "${cpp_typesupport_target}"
)

install(TARGETS
  waypoint_manager
  mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/interactive_waypoints.py
  scripts/generate_inventory_mission.py
  scripts/optimize_inventory_route.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
