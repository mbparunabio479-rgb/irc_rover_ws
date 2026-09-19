cmake_minimum_required(VERSION 3.8)
project(irc_rover_terrain_features)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

add_executable(terrain_features
  src/terrain_features.cpp
)

ament_target_dependencies(
  terrain_features
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

install(
  TARGETS terrain_features
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
