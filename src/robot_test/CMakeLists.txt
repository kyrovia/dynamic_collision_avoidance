cmake_minimum_required(VERSION 3.8)
project(robot_test)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS robot_test/reachability_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reachability_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
