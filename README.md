# hikrobot_ros2

ROS 2 Humble driver for a Hikrobot HAR robot using the public MPSDK legacy motion API.

## Verified hardware baseline

- Ubuntu 22.04 / ROS 2 Humble target
- Controller observed in HIL tests: `V2.1.2_1444184_20241223`
- SDK package used during bring-up: `MPSDK_PUBLIC_LX64_V2.4.0_Build20251106_SVN1686294`
- Link library: `libMPSDK_PUBLIC_COMPAT.so`
- Motion API family: `MP_Motion_API.h` (legacy realtime motion API)
- HIL verified: connection, HAR state subscription, control access, programmer login, AUTO, MotorOn/Off, realtime mode, MoveJ, common DO read/write
- SDK/demo-backed and implemented in this repository, pending individual HIL regression: MoveL, MoveC, MoveS, DI read

## Build

Install ROS 2 Humble and the standard message packages, then point CMake at the Hikrobot SDK:

```bash
export HIKROBOT_MPSDK_ROOT=/path/to/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
# The vendor package also installs an environment hook for this directory.
# Before the first colcon build, exporting it explicitly is useful for direct tests:
export LD_LIBRARY_PATH="$HIKROBOT_MPSDK_ROOT/Libs/Linux/LX64/64:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
cd ~/hikrobot_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`HIKROBOT_MPSDK_ROOT` must contain `Include/Public` and `Libs/Linux/LX64/64/libMPSDK_PUBLIC_COMPAT.so`.

## Runtime password

The driver reads the programmer password from the environment variable configured by `programmer_password_env` (default `HIKROBOT_PROGRAMMER_PASSWORD`). It is not stored in YAML.

## Motion model

Every motion uses the controller-side legacy queue model:

`MP_ClearMotionTarget -> MP_AddMov*Target -> MP_StartRealTimeMotion`

`nPathID` is allocated internally and persisted before the SDK Add call. Error `0x82068043` is treated as a recoverable first-command sequence collision only before motion starts; the core reserves a new ID and retries a bounded number of times.

See `docs/ARCHITECTURE.md` for the detailed state machine and safety rules.
