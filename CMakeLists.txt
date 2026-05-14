cmake_minimum_required(VERSION 3.8)
project(vacuum_game)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


include_directories(include)

add_executable(vacuum_node
  src/nodes/VacuumNode.cpp
  src/core/Robot.cpp
  src/core/Lidar.cpp
  src/core/Environment.cpp
)

add_executable(visualization_node
  src/nodes/VisualizationNode.cpp
)

add_executable(game_node
  src/nodes/GameNode.cpp
)

ament_target_dependencies(vacuum_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  ament_index_cpp
)


ament_target_dependencies(visualization_node
  rclcpp
  visualization_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

ament_target_dependencies(game_node
  rclcpp
)

install(TARGETS
  vacuum_node
  game_node
  visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
