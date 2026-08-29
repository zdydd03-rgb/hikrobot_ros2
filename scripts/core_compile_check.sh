#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${HIKROBOT_MPSDK_ROOT:?Set HIKROBOT_MPSDK_ROOT}"
INC="$SDK_ROOT/Include/Public"
LIB="$SDK_ROOT/Libs/Linux/LX64/64"
BUILD="$ROOT/_build_core_check"
rm -rf "$BUILD"
mkdir -p "$BUILD"

CXXFLAGS=(-std=c++17 -fPIC -Wall -Wextra -Wpedantic \
  -I"$ROOT/src/hikrobot_mpsdk_core/include" -I"$INC")

g++ "${CXXFLAGS[@]}" -c "$ROOT/src/hikrobot_mpsdk_core/src/motion_sequence_manager.cpp" -o "$BUILD/sequence.o"
g++ "${CXXFLAGS[@]}" -c "$ROOT/src/hikrobot_mpsdk_core/src/robot_runtime.cpp" -o "$BUILD/runtime.o"
g++ -shared "$BUILD/sequence.o" "$BUILD/runtime.o" \
  -L"$LIB" -Wl,-rpath,"$LIB" -lMPSDK_PUBLIC_COMPAT -lpthread -ldl \
  -o "$BUILD/libhikrobot_mpsdk_core.so"

echo "Core compile/link check: PASS"
