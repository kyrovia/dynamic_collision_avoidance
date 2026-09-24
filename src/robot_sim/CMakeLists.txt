cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS robot_sim/sphere_motion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sphere_motion_node
)

install(
  DIRECTORY config launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
