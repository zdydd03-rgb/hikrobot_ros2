cmake_minimum_required(VERSION 3.8)
project(hikrobot_mpsdk_vendor)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# ------------------------------------------------------------
# Locate Hikrobot MPSDK
# ------------------------------------------------------------

set(
  HIKROBOT_MPSDK_ROOT
  "$ENV{HIKROBOT_MPSDK_ROOT}"
  CACHE PATH
  "Root directory of Hikrobot MPSDK"
)

if(NOT HIKROBOT_MPSDK_ROOT)
  message(FATAL_ERROR
    "HIKROBOT_MPSDK_ROOT is not set.\n"
    "Expected SDK layout:\n"
    "  <root>/Include/Public\n"
    "  <root>/Libs/Linux/LX64/64\n"
  )
endif()

set(
  HIKROBOT_MPSDK_INCLUDE_DIR
  "${HIKROBOT_MPSDK_ROOT}/Include/Public"
)

set(
  HIKROBOT_MPSDK_LIBRARY_DIR
  "${HIKROBOT_MPSDK_ROOT}/Libs/Linux/LX64/64"
)

set(
  HIKROBOT_MPSDK_LIBRARY
  "${HIKROBOT_MPSDK_LIBRARY_DIR}/libMPSDK_PUBLIC_COMPAT.so"
)

# ------------------------------------------------------------
# Validate SDK
# ------------------------------------------------------------

if(NOT EXISTS "${HIKROBOT_MPSDK_INCLUDE_DIR}/MP_Public_API.h")
  message(FATAL_ERROR
    "MP_Public_API.h not found:\n"
    "  ${HIKROBOT_MPSDK_INCLUDE_DIR}/MP_Public_API.h"
  )
endif()

if(NOT EXISTS "${HIKROBOT_MPSDK_INCLUDE_DIR}/MP_Motion_API.h")
  message(FATAL_ERROR
    "MP_Motion_API.h not found:\n"
    "  ${HIKROBOT_MPSDK_INCLUDE_DIR}/MP_Motion_API.h"
  )
endif()

if(NOT EXISTS "${HIKROBOT_MPSDK_LIBRARY}")
  message(FATAL_ERROR
    "libMPSDK_PUBLIC_COMPAT.so not found:\n"
    "  ${HIKROBOT_MPSDK_LIBRARY}"
  )
endif()

message(STATUS "Hikrobot MPSDK root    : ${HIKROBOT_MPSDK_ROOT}")
message(STATUS "Hikrobot MPSDK include : ${HIKROBOT_MPSDK_INCLUDE_DIR}")
message(STATUS "Hikrobot MPSDK library : ${HIKROBOT_MPSDK_LIBRARY}")

# ------------------------------------------------------------
# Exported MPSDK interface target
# ------------------------------------------------------------

add_library(hikrobot_mpsdk INTERFACE)

add_library(
  hikrobot_mpsdk_vendor::hikrobot_mpsdk
  ALIAS hikrobot_mpsdk
)

target_include_directories(
  hikrobot_mpsdk
  INTERFACE
    "${HIKROBOT_MPSDK_INCLUDE_DIR}"
)

target_link_libraries(
  hikrobot_mpsdk
  INTERFACE
    "${HIKROBOT_MPSDK_LIBRARY}"
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

# Help the linker locate secondary Hikrobot SDK dependencies.
target_link_options(
  hikrobot_mpsdk
  INTERFACE
    "-Wl,-rpath-link,${HIKROBOT_MPSDK_LIBRARY_DIR}"
)

# ------------------------------------------------------------
# Install/export target
# ------------------------------------------------------------

install(
  TARGETS hikrobot_mpsdk
  EXPORT export_hikrobot_mpsdk
)

install(
  EXPORT export_hikrobot_mpsdk
  NAMESPACE hikrobot_mpsdk_vendor::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ------------------------------------------------------------
# Runtime library environment
# ------------------------------------------------------------

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/hikrobot_mpsdk_library_path.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hikrobot_mpsdk_library_path.sh"
  @ONLY
)

ament_environment_hooks(
  "${CMAKE_CURRENT_BINARY_DIR}/hikrobot_mpsdk_library_path.sh"
)

ament_export_targets(
  export_hikrobot_mpsdk
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(Threads)

ament_package()
