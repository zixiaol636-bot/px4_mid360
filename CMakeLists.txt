cmake_minimum_required(VERSION 3.8)
project(px4_mid360)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  docs
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  CLAUDE.md
  dependencies.repos
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
