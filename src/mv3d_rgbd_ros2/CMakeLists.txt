cmake_minimum_required(VERSION 3.5)
project(mv3d_rgbd_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Hikrobot Mv3dRgbd SDK
# ============================================================

set(MV3D_RGBD_SDK_DIRS "/opt/Mv3dRgbdSDK")

execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCH_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(ARCH_OUTPUT MATCHES "x86_64|amd64")
  set(HOST_PLATFORM "linux64")
elseif(ARCH_OUTPUT MATCHES "i686")
  set(HOST_PLATFORM "linux32")
elseif(ARCH_OUTPUT MATCHES "aarch64")
  set(HOST_PLATFORM "aarch64")
else()
  message(FATAL_ERROR "Unsupported host architecture: ${ARCH_OUTPUT}")
endif()

# ============================================================
# ROS2 dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(mv3d_rgbd_ros2_interface REQUIRED)

# ============================================================
# SDK include / library
# ============================================================

include_directories(
  ${MV3D_RGBD_SDK_DIRS}/include
)

find_library(
  MV3D_RGBD_SDK_LIB
  NAMES Mv3dRgbd
  PATHS ${MV3D_RGBD_SDK_DIRS}/lib/${HOST_PLATFORM}
  NO_DEFAULT_PATH
)

if(NOT MV3D_RGBD_SDK_LIB)
  message(FATAL_ERROR
    "Mv3dRgbd SDK library not found in "
    "${MV3D_RGBD_SDK_DIRS}/lib/${HOST_PLATFORM}"
  )
endif()

message(STATUS "Mv3dRgbd SDK root: ${MV3D_RGBD_SDK_DIRS}")
message(STATUS "Mv3dRgbd platform: ${HOST_PLATFORM}")
message(STATUS "Mv3dRgbd library: ${MV3D_RGBD_SDK_LIB}")

# ============================================================
# Image publisher
# ============================================================

add_executable(
  hik_camera_image_pub
  src/create_camera_image_pub.cpp
  src/hik_camera.cpp
)

target_link_libraries(
  hik_camera_image_pub
  ${MV3D_RGBD_SDK_LIB}
)

ament_target_dependencies(
  hik_camera_image_pub
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  camera_info_manager
  OpenCV
  PCL
)

# ============================================================
# Image subscriber
# ============================================================

add_executable(
  hik_camera_image_sub
  src/create_camera_image_sub.cpp
)

ament_target_dependencies(
  hik_camera_image_sub
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  camera_info_manager
  OpenCV
  PCL
)

# ============================================================
# Image service client
# ============================================================

add_executable(
  hik_camera_image_client
  src/create_camera_image_client.cpp
  src/hik_camera.cpp
)

target_link_libraries(
  hik_camera_image_client
  ${MV3D_RGBD_SDK_LIB}
)

ament_target_dependencies(
  hik_camera_image_client
  rclcpp
  sensor_msgs
  mv3d_rgbd_ros2_interface
  cv_bridge
  pcl_conversions
  camera_info_manager
  OpenCV
  PCL
)

# ============================================================
# Image service server
# ============================================================

add_executable(
  hik_camera_image_server
  src/create_camera_image_server.cpp
)

ament_target_dependencies(
  hik_camera_image_server
  rclcpp
  sensor_msgs
  mv3d_rgbd_ros2_interface
  cv_bridge
  pcl_conversions
  camera_info_manager
  OpenCV
  PCL
)

# ============================================================
# Point cloud service client
# ============================================================

add_executable(
  hik_camera_point_cloud_client
  src/create_camera_point_cloud_client.cpp
  src/hik_camera.cpp
)

target_link_libraries(
  hik_camera_point_cloud_client
  ${MV3D_RGBD_SDK_LIB}
)

ament_target_dependencies(
  hik_camera_point_cloud_client
  rclcpp
  sensor_msgs
  mv3d_rgbd_ros2_interface
  cv_bridge
  pcl_conversions
  camera_info_manager
  OpenCV
  PCL
)

# ============================================================
# Point cloud service server
# ============================================================

add_executable(
  hik_camera_point_cloud_server
  src/create_camera_point_cloud_server.cpp
)

ament_target_dependencies(
  hik_camera_point_cloud_server
  rclcpp
  sensor_msgs
  mv3d_rgbd_ros2_interface
  cv_bridge
  pcl_conversions
  camera_info_manager
  OpenCV
  PCL
)

# ============================================================
# Install
# ============================================================

install(
  TARGETS
    hik_camera_image_pub
    hik_camera_image_sub
    hik_camera_image_client
    hik_camera_image_server
    hik_camera_point_cloud_client
    hik_camera_point_cloud_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
