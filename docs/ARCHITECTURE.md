# Architecture

## Packages

- `hikrobot_mpsdk_vendor`: exposes the external MPSDK headers and `libMPSDK_PUBLIC_COMPAT.so` as a CMake target.
- `hikrobot_msgs`: custom actions, services, and status messages.
- `hikrobot_mpsdk_core`: ROS-independent C++17 driver core.
- `hikrobot_driver`: ROS 2 Humble lifecycle node, actions, services, and state publishers.
- `hikrobot_bringup`: launch and YAML configuration.

## Lifecycle

`configure`:
1. create SDK handle
2. connect controller
3. subscribe HAR robot status
4. wait for the first valid state
5. record original work mode

`activate`:
1. request control access
2. switch to PROGRAMMER
3. eliminate alarms
4. register legacy motion callback
5. MotorOff and confirm
6. set AUTO and verify
7. MotorOn and confirm
8. enter realtime motion mode
9. set equilibrium motion mode

`deactivate`:
1. reject new goals
2. request cancellation of the active goal
3. `MP_StopRealTimeMotion`
4. `MP_ExitRealTimeMotionMode`
5. MotorOff
6. restore original work mode
7. release control access

`cleanup`:
1. unsubscribe HAR state
2. disconnect
3. destroy SDK handle

## Motion execution

The core serializes all motion commands. MoveJ, MoveL, MoveC and MoveS share the same executor and differ only in SDK parameter construction and completion criteria.

### Base Cartesian contract

MoveL targets, MoveC via/target poses, and every MoveS pose are `^B T_TCP` for the controller
Active Tool. Only the exact configured `base_frame` is accepted; empty, camera, flange, Tool,
and other frames are rejected without TF conversion. Before any Cartesian SDK Add call or
PathID reservation, the Driver refreshes the Active Tool through `RobotRuntime`'s existing
handle and SDK mutex, requires a flange mount, snapshots the Tool, and forces legacy motion
parameters to Base `nCsType:nCsID = 1:0`, `nToolId = active Tool ID`, and `nWorkPieceId = -1`.
The legacy YAML `tool_id` remains declared for compatibility but does not control Cartesian
Actions; MoveJ behavior is unchanged.

The same Tool snapshot is retained for the whole request. HAR supplies actual `^B T_F`, so
MoveL/MoveC/MoveS feedback and stable completion use
`^B T_TCP_actual = ^B T_F_actual × ^F T_Tool_snapshot`. Their target and actual feedback are
therefore both TCP, not flange. ROS input/output uses m and normalized quaternion; SDK targets
use mm and ZYX Euler degrees.

A pre-Add software guard checks current TCP to the first requested point and every adjacent
MoveC/MoveS segment. Defaults are 0.05 m and 0.1745329252 rad (10 degrees); any violation,
invalid/stale HAR state, invalid Tool, or Tool-read failure aborts before core execution, so no
Add API is called and no PathID is consumed. The initial V0.2-03 HIL scope is a current-TCP Base
Z +10 mm MoveL and return only. Camera/hand-eye input and TF conversion are not implemented.

### Sequence collision recovery

The controller V2.1.2 HIL logs showed that a previously executed `nPathID` can survive client process exit. Reusing the same first motion sequence can make `MP_AddMov*Target` return `0x82068043` (`HKR_E_IN_SYS_CALL_REMOTE_FIRET_CMD_REPEAT`).

The driver therefore:
- never accepts a user supplied `nPathID`;
- uses one `MotionSequenceManager` shared by all motion types;
- reserves and persists IDs before calling the SDK;
- retries only the Add stage when the exact error is `0x82068043`;
- never automatically repeats a motion after `MP_StartRealTimeMotion` has been attempted successfully, because execution may already have started.

### FollowCartesianPath (software-only)

`follow_cartesian_path` accepts 1..32 ordered `base_frame` Active-Tool TCP poses. The driver
validates every pose, the timeout and the configured 50 mm/10 degree per-segment guard before
allocating IDs. The core then reserves and persists two IDs per point, clears the legacy queue,
adds each `ST_MOTION_MOV_PARAM` with Base coordinate `1:0`, `nToolId` from one Active Tool
 snapshot and `nWorkPieceId=-1`. Controller 2.1.2 HIL observed a ten-entry active Legacy window:
the core adds the first ten targets, calls `MP_StartRealTimeMotion` exactly once, then refills
vacancies from new cached Motion callback sequences using `nLeftPathNum`. The Legacy header does
not state the limit; API2's documented ten-entry queue is corroboration only and is not used as a
calling interface. The Action is not split, every target is added once with its original
preallocated PathID, and normal execution issues no second Start or Clear. A collision retry is
permitted only before Start and requeues the complete path with a fresh persisted block. A stopped
callback with `left_paths>0`, unsubmitted targets, or no final PathID is queue underflow/premature
stop, not success.

Feedback and completion use the same Tool snapshot to compose HAR `^B T_F` into
`^B T_TCP = ^B T_F × ^F T_Tool`. Completion requires post-Start matching motion-callback queue
evidence, distinct HAR samples, a stopped terminal callback referencing the final target, and
TCP tolerance. Progress percentage is deliberately not synthesized from time or `dPercent`.
Cancellation, StopMotion and lifecycle stops are serialized with Start and map to the existing
Action CANCELED/ABORTED semantics. Start uncertainty or failed queue cleanup blocks subsequent
motion until lifecycle cleanup. This implementation is not yet validated against controller
2.1.2 for continuous queue behavior; MoveC/MoveS and online visual correction remain separate.

## Units

ROS boundary:
- joints: rad
- translation: m
- orientation: quaternion

MPSDK boundary:
- rotary joints: degree
- translation: mm
- orientation: ZYX Euler degree

HAR completion feedback uses `ST_HAR_ROBOT_STATUS::dJnt[]` and `stBasePose`.
`stBasePose` is the base-to-flange transform `^B T_F`; it is published as
`/hikrobot_driver/flange_pose`, not as a Tool TCP pose. V0.2-01 uses the ROS publish time
for its timestamp; mapping HAR/controller sampling time is deferred to timestamp-discipline work.

The active Tool is read through the controller's read-only Tool APIs and cached by the Driver.
`^F T_Tool` is converted from SDK mm/ZYX-degree Euler data to ROS m/quaternion, then TCP is
computed directly from the same HAR snapshot as `^B T_TCP = ^B T_F × ^F T_Tool`. External-mounted
Tools and invalid caches suppress TCP publication. No Tool write/switch API or TF broadcaster exists.
