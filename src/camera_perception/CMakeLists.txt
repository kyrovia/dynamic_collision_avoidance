cmake_minimum_required(VERSION 3.8)
project(camera_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClosestObstacle.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(aabb_crop src/aabb_crop.cpp)
target_include_directories(aabb_crop SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(aabb_crop PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(aabb_crop ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(aabb_crop ament_index_cpp)

add_executable(obstacle_cloud_node src/obstacle_cloud_node.cpp)
target_include_directories(obstacle_cloud_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_options(obstacle_cloud_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(obstacle_cloud_node ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_cloud_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

add_executable(closest_obstacle_node src/closest_obstacle_node.cpp)
ament_target_dependencies(closest_obstacle_node
  rclcpp
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(closest_obstacle_node "${cpp_typesupport_target}")

install(TARGETS aabb_crop obstacle_cloud_node closest_obstacle_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config data launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
