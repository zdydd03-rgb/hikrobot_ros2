cmake_minimum_required(VERSION 3.8)
project(hikrobot_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hikrobot_msgs REQUIRED)
find_package(hikrobot_mpsdk_core REQUIRED)

add_executable(hikrobot_driver_node
  src/hikrobot_driver_node.cpp
  src/main.cpp
)
target_compile_features(hikrobot_driver_node PUBLIC cxx_std_17)
target_compile_options(hikrobot_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(hikrobot_driver_node PRIVATE include)
ament_target_dependencies(hikrobot_driver_node
  rclcpp builtin_interfaces rclcpp_lifecycle rclcpp_action
  sensor_msgs geometry_msgs diagnostic_msgs
  tf2 tf2_ros tf2_geometry_msgs
  hikrobot_msgs hikrobot_mpsdk_core
)

install(TARGETS hikrobot_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
