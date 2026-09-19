cmake_minimum_required(VERSION 3.8)
project(irc_rover_terrain_costmap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

include_directories(
  include
)

add_library(terrain_costmap_layer SHARED
  src/terrain_costmap_layer.cpp
)

ament_target_dependencies(
  terrain_costmap_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

target_include_directories(
  terrain_costmap_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  plugin.xml
)

install(
  TARGETS terrain_costmap_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
