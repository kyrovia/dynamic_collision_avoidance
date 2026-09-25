cmake_minimum_required(VERSION 3.8)
project(robot_avoidance_apf)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS robot_avoidance_apf/apf_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME apf_node
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_apf test/test_apf.py)
  ament_add_pytest_test(test_kinematics test/test_kinematics.py)
endif()

ament_package()
