cmake_minimum_required(VERSION 3.8)
project(hikrobot_mpsdk_core)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(hikrobot_mpsdk_vendor REQUIRED)

add_library(
  hikrobot_mpsdk_core_logic
  STATIC
    src/motion_sequence_manager.cpp
    src/legacy_movel_queue_executor.cpp
    src/command_channel_health.cpp
    src/protected_arc_output.cpp
    src/tool_transform.cpp
)

target_compile_features(
  hikrobot_mpsdk_core_logic
  PUBLIC
    cxx_std_17
)

target_compile_options(
  hikrobot_mpsdk_core_logic
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

target_include_directories(
  hikrobot_mpsdk_core_logic
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set_target_properties(hikrobot_mpsdk_core_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(hikrobot_mpsdk_core_logic PUBLIC Threads::Threads)

add_library(${PROJECT_NAME} SHARED src/robot_runtime.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
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
  PRIVATE
    hikrobot_mpsdk_core_logic
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
  target_link_libraries(test_tool_transform hikrobot_mpsdk_core_logic)
  ament_add_gtest(test_command_channel_health tests/test_command_channel_health.cpp)
  target_link_libraries(test_command_channel_health hikrobot_mpsdk_core_logic)
  ament_add_gtest(test_protected_arc_output tests/test_protected_arc_output.cpp)
  target_link_libraries(test_protected_arc_output hikrobot_mpsdk_core_logic)
  ament_add_gtest(
    test_legacy_movel_queue_executor
    tests/test_legacy_movel_queue_executor.cpp
  )
  target_link_libraries(test_legacy_movel_queue_executor hikrobot_mpsdk_core_logic)
endif()

ament_package()
