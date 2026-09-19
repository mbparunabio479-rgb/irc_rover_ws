cmake_minimum_required(VERSION 3.8)
project(irc_rover_terrain)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(terrain_preprocessor
  src/terrain_preprocessor.cpp
)

ament_target_dependencies(
  terrain_preprocessor
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

target_link_libraries(
  terrain_preprocessor
  ${PCL_LIBRARIES}
)

install(
  TARGETS terrain_preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
