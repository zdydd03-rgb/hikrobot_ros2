#!/usr/bin/env bash

set -eo pipefail

# ROS 2 setup scripts are not guaranteed to be compatible with nounset.
set +u
source /opt/ros/humble/setup.bash
set -u

: "${HIKROBOT_MPSDK_ROOT:?HIKROBOT_MPSDK_ROOT is not set}"

MPSDK_INC="${HIKROBOT_MPSDK_ROOT}/Include/Public"
MPSDK_LIB="${HIKROBOT_MPSDK_ROOT}/Libs/Linux/LX64/64"

if [[ ! -f "${MPSDK_INC}/MP_Public_API.h" ]]; then
  echo "[ERROR] MP_Public_API.h not found:"
  echo "        ${MPSDK_INC}/MP_Public_API.h"
  exit 1
fi

if [[ ! -f "${MPSDK_LIB}/libMPSDK_PUBLIC_COMPAT.so" ]]; then
  echo "[ERROR] libMPSDK_PUBLIC_COMPAT.so not found:"
  echo "        ${MPSDK_LIB}/libMPSDK_PUBLIC_COMPAT.so"
  exit 1
fi

export LD_LIBRARY_PATH="${MPSDK_LIB}:${LD_LIBRARY_PATH:-}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

echo "=========================================="
echo " Hikrobot ROS 2 Humble Build"
echo "=========================================="
echo "Workspace  : ${ROOT}"
echo "MPSDK root : ${HIKROBOT_MPSDK_ROOT}"
echo "MPSDK inc  : ${MPSDK_INC}"
echo "MPSDK lib  : ${MPSDK_LIB}"
echo "=========================================="

colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
