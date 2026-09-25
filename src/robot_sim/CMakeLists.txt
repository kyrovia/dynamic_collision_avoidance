cmake_minimum_required(VERSION 3.8)
project(robot_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS robot_sim/sphere_motion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sphere_motion_node
)

install(
  PROGRAMS robot_sim/computed_torque_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME computed_torque_node
)

install(
  DIRECTORY config launch urdf worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dynamics test/test_dynamics.py)
endif()

ament_package()
