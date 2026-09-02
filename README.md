# hikrobot_ros2

ROS 2 Humble drivers for a Hikrobot HAR robot and Hikrobot RGB-D cameras, using
the public MPSDK legacy motion API and Mv3dRgbdSDK respectively.

## Verified hardware baseline

- Ubuntu 22.04 / ROS 2 Humble target
- Controller observed in HIL tests: `V2.1.2_1444184_20241223`
- SDK package used during bring-up: `MPSDK_PUBLIC_LX64_V2.4.0_Build20251106_SVN1686294`
- Link library: `libMPSDK_PUBLIC_COMPAT.so`
- Motion API family: `MP_Motion_API.h` (legacy realtime motion API)
- HIL verified: connection, HAR state subscription, control access, programmer login, AUTO, MotorOn/Off, realtime mode, MoveJ, common DO read/write
- SDK/demo-backed and implemented in this repository, pending individual HIL regression: MoveL, MoveC, MoveS, DI read

## ROS packages

In addition to the robot motion driver, this workspace contains the following
Hikrobot RGB-D camera packages:

### `mv3d_rgbd_ros2`

ROS 2 wrapper and example nodes for the Hikrobot Mv3dRgbdSDK. The package
connects to a camera by serial number and provides:

- depth, RGB, left IR, and right IR image acquisition and publication;
- depth images aligned to the RGB optical frame through the SDK
  `MV3D_RGBD_ImageCoordinateTrans` API;
- depth and RGB `sensor_msgs/msg/CameraInfo` publication;
- point-cloud acquisition for XYZ, XYZ + normals, XYZRGB, and XYZRGB + normals;
- image and point-cloud save clients and servers; and
- launch files for a single camera, multiple cameras, image capture, and point
  cloud acquisition.

The image publisher executable is `hik_camera_image_pub`. Its default image
topics are `/camera/depth`, `/camera/rgb`, `/camera/aligned_depth`,
`/camera/leftIr`, and `/camera/rightIr`. The aligned-depth topic can be changed
with the `aligned_depth_topic` parameter. Aligned depth uses `mono16` encoding,
the RGB image dimensions, and the RGB optical frame ID.

This package expects the Hikrobot Mv3dRgbdSDK under `/opt/Mv3dRgbdSDK` and
selects the SDK library directory for `x86_64`, `i686`, or `aarch64` during the
CMake configure step.

### `mv3d_rgbd_ros2_interface`

ROS 2 interface-only package used by `mv3d_rgbd_ros2`. It generates two custom
services:

- `Image.srv`: transfers the camera serial number, image-type mask, frame
  number, timestamp, camera calibration, and acquired images to the image-save
  service.
- `PointCloud.srv`: transfers the camera serial number, point-cloud type, frame
  number, timestamp, and `sensor_msgs/msg/PointCloud2` data to the point-cloud
  save service.

### RGB-D hardware verification

The RGB-D camera integration and the current depth-to-RGB alignment path have
been verified successfully on physical hardware. The verified alignment flow
is `Start -> FetchFrame -> GetCameraParam -> ImageCoordinateTrans`; calibration
dimensions are checked against the acquired depth and RGB frames before each
coordinate conversion.

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
