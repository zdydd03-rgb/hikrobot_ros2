# Hikrobot Tool read-only HIL probe

This is a standalone C++17 MPSDK probe. It is not a ROS node and does not integrate with
the ROS Driver or a ToolManager. It only tests whether the controller permits the following
official read APIs after connection, without control access or any controller-state change:

- `MP_GetActivateTool()`
- `MP_HAR_GetToolInfoList()`
- `MP_HAR_GetToolInfo()`

Before running it, ensure the ROS Driver has released its SDK connection:

```bash
ros2 lifecycle set /hikrobot_driver deactivate
ros2 lifecycle set /hikrobot_driver cleanup
ros2 lifecycle get /hikrobot_driver
```

Then stop the Driver process and confirm there is no other MPSDK connection owner.

## Build

```bash
MPSDK_ROOT=/home/ai/SDK/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
MPSDK_INC="$MPSDK_ROOT/Include/Public"
MPSDK_LIB="$MPSDK_ROOT/Libs/Linux/LX64/64"

g++ -std=c++17 -Wall -Wextra -Wpedantic \
  main.cpp \
  -I"$MPSDK_INC" \
  -L"$MPSDK_LIB" \
  -Wl,-rpath,"$MPSDK_LIB" \
  -lMPSDK_PUBLIC_COMPAT \
  -pthread -ldl \
  -o tool_readonly_probe
```

## Run

The executable requires an explicit controller address and port; it has no default address.

```bash
./tool_readonly_probe 192.168.2.64 9000
```

The current HIL expectation for manual comparison is:

```text
active_tool_id = 2
XYZ_mm  ~= 137.521, 1.497, 187.672
RPY_deg ~= 0.015, -0.096, 0.010
```

The program prints the decimal and low-32-bit hexadecimal return value for every SDK call.
Any SDK error exits non-zero. It never requests control access, changes user/mode/motor state,
switches or writes a tool, sends motion, or writes IO.
