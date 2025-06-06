cmake_minimum_required(VERSION 3.10)
project(costmap)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# Search for dependencies required for building this package
find_package(ament_cmake REQUIRED) # ROS2 build tool
find_package(rclcpp REQUIRED)      # ROS2 C++ package
find_package(std_msgs REQUIRED)    # Standard ROS2 messages
find_package(sensor_msgs REQUIRED) # Sensor messages for LaserScan
find_package(nav_msgs REQUIRED)    # Navigation messages for OccupancyGrid
find_package(geometry_msgs REQUIRED) # Geometry messages for Pose

add_library(costmap_lib
  src/costmap_core.cpp)
target_include_directories(costmap_lib
  PUBLIC include)
# Add ROS2 dependencies required by package
ament_target_dependencies(costmap_lib 
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

add_executable(costmap_node src/costmap_node.cpp)
target_link_libraries(costmap_node costmap_lib)
ament_target_dependencies(costmap_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  costmap_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})
 
ament_package()
