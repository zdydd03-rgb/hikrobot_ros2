cmake_minimum_required(VERSION 3.8)
project(hikrobot_mpsdk_core)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(hikrobot_mpsdk_vendor REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
    src/motion_sequence_manager.cpp
    src/robot_runtime.cpp
    src/tool_transform.cpp
)

target_compile_features(
  ${PROJECT_NAME}
  PUBLIC
    cxx_std_17
)

target_compile_options(
  ${PROJECT_NAME}
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    hikrobot_mpsdk_vendor::hikrobot_mpsdk
    Threads::Threads
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  hikrobot_mpsdk_vendor
)
ament_export_dependencies(Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tool_transform tests/test_tool_transform.cpp)
  target_link_libraries(test_tool_transform ${PROJECT_NAME})
endif()

ament_package()
