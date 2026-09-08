# Implementation status — V0.2-03

## Implemented

- ROS 2 Humble package layout
- external MPSDK vendor target and runtime `LD_LIBRARY_PATH` environment hook
- ROS-independent `RobotRuntime`
- exact tested activation chain: control access -> PROGRAMMER -> clear alarms -> motion callback -> MotorOff -> AUTO -> MotorOn -> realtime -> equilibrium mode
- HAR state cache and stale-state protection
- persistent `MotionSequenceManager` with persist-before-send semantics
- bounded automatic recovery for `0x82068043` during the Add stage only
- shared MotionExecutor path for MoveJ / MoveL / MoveC / MoveS
- MoveJ rad->deg conversion and joint-CS semantics matching the HIL 05D baseline
- Cartesian m/quaternion -> mm/ZYX-Euler-degree conversion
- MoveC via/final mapping from the official SDK structure
- MoveS via array mapping with the public 49-point limit
- Action cancellation -> `MP_StopRealTimeMotion`
- no automatic resend after `MP_StartRealTimeMotion`
- DO write/read based on the HIL 06D path; DI read from the same public IO API
- lifecycle driver publishers/services/actions
- automatic lifecycle bringup configuration and activation
- HIL regression plan
- lifecycle-aware `/flange_pose` (`^B T_F`) and `/tcp_pose` (`^B T_TCP`) publication
- read-only controller Active Tool query, cache, and ROS services
- strict Base-frame Active-Tool TCP contract for MoveL, MoveC, and MoveS
- per-segment 50 mm / 10 degree Cartesian pre-Add guards
- TCP-based Cartesian feedback and stable completion using a per-motion Tool snapshot

## Verification completed

The Ubuntu 22.04 + ROS 2 Humble development machine completed a full workspace build:

- all six ROS 2 packages passed `./scripts/build_humble.sh` / `colcon build`
- `hikrobot_mpsdk_core` compiled with C++17, `-Wall -Wextra -Wpedantic`
- core linked successfully against the supplied `libMPSDK_PUBLIC_COMPAT.so`
- runtime dependency resolution verified for `libMPSDK_ADAPTER.so`, `libMPSDK_LOG.so`, `libMPSDKV212.so`, `libMPSDKV220.so`
- PathID persistence unit test passed across a simulated process restart
- launch Python syntax checked
- all `package.xml` files parsed as XML
- all six packages were discoverable through the ROS 2 package index
- `hikrobot_driver_node` started without a robot connection; its Lifecycle, topics,
  services, and MoveJ/MoveL/MoveC/MoveS action servers were visible in the ROS graph
- physical HIL passed for Configure/HAR, Activate, MoveJ, Cancel, StopMotion, PathID persistence,
  sequence-collision recovery, flange/TCP semantics, Tool queries, Base TCP MoveL, invalid-frame
  rejection, the 50 mm guard, and a request-specific segmented visual MoveL route through D7

## Pending or deferred

- IO/DO HIL is deferred.
- MoveC and MoveS have software tests but no physical HIL.
- The complete automated visual three-point route and MoveL speed 3%/5% are not verified.
- `FollowCartesianPath`, a dynamic visual-coordinate transform node, image-time/historical
  flange-pose synchronization, and Driver hand-eye integration are not implemented.

See `docs/hil/` for reviewed HIL evidence and exact limitations.
