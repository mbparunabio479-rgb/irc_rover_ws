cmake_minimum_required(VERSION 3.8)
project(irc_rover_obstacle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(obstacle_filter
  src/obstacle_filter.cpp
)

ament_target_dependencies(
  obstacle_filter
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

install(
  TARGETS obstacle_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
