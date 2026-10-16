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
find_package(tf2_geometry_msgs REQUIRED)
find_package(hikrobot_msgs REQUIRED)
find_package(hikrobot_mpsdk_core REQUIRED)

add_library(hikrobot_cartesian_contract src/cartesian_contract.cpp)
target_compile_features(hikrobot_cartesian_contract PUBLIC cxx_std_17)
target_compile_options(hikrobot_cartesian_contract PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(hikrobot_cartesian_contract PUBLIC include)
ament_target_dependencies(hikrobot_cartesian_contract
  geometry_msgs tf2 tf2_geometry_msgs hikrobot_mpsdk_core)

add_library(hikrobot_health_monitor src/health_monitor.cpp)
target_compile_features(hikrobot_health_monitor PUBLIC cxx_std_17)
target_compile_options(hikrobot_health_monitor PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(hikrobot_health_monitor PUBLIC include)
ament_target_dependencies(hikrobot_health_monitor hikrobot_mpsdk_core)

add_executable(hikrobot_driver_node src/hikrobot_driver_node.cpp src/main.cpp)
target_compile_features(hikrobot_driver_node PUBLIC cxx_std_17)
target_compile_options(hikrobot_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(hikrobot_driver_node PRIVATE include)
ament_target_dependencies(hikrobot_driver_node
  rclcpp builtin_interfaces rclcpp_lifecycle rclcpp_action
  sensor_msgs geometry_msgs diagnostic_msgs
  tf2 tf2_geometry_msgs
  hikrobot_msgs hikrobot_mpsdk_core
)
target_link_libraries(hikrobot_driver_node hikrobot_cartesian_contract hikrobot_health_monitor)

install(
  TARGETS hikrobot_driver_node hikrobot_cartesian_contract hikrobot_health_monitor
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cartesian_contract tests/test_cartesian_contract.cpp)
  target_link_libraries(test_cartesian_contract hikrobot_cartesian_contract)
  ament_target_dependencies(test_cartesian_contract geometry_msgs tf2 hikrobot_mpsdk_core)
  ament_add_gtest(test_health_monitor tests/test_health_monitor.cpp)
  target_link_libraries(test_health_monitor hikrobot_health_monitor)
  ament_target_dependencies(test_health_monitor hikrobot_mpsdk_core)
endif()

ament_package()
