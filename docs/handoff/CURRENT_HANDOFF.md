# Hikrobot ROS 2 current handoff

Handoff generated: 2026-09-09 (Asia/Shanghai)

Purpose: establish the repository truth needed for later IO HIL and automatic continuous visual
trajectory work. This document records the current implementation and reviewed historical evidence;
it does not authorize or report a new controller connection or robot operation.

## Current entry point — 2026-10-24

**IO HIL succeeded on 2026-10-24; the eleven-point MoveL queue HIL failed**, per the operator.
Current first-version usage is **1..10 points only**. The unchanged source still accepts 32;
applications must apply the ten-point limit. Larger fake-backend tests are not supported HIL scope.
See [current status](../hil/V1_STATUS_2026-10-24.md) and [queue usage](../FOLLOW_CARTESIAN_PATH.md).
The operator did not supply raw logs or exact hardware-test build identity; no failure cause is inferred.

Current production source baseline is `ae33fb7`; the earlier “uncommitted” statements below
refer to their dated historical snapshots, not today's Git state. This documentation update does
not include the untracked local visual scripts, hand-eye calibration sample or SDK runtime logs.
Read the current status first, then the [2026-10-16 code audit](HIKROBOT_DRIVER_FINAL_HIL_CODE_HANDOFF.md)
for detailed interfaces and risks. The remaining dated sections retain historical evidence.

## 2026-09-14 FollowCartesianPath Legacy queue-window repair

**[User HIL report plus source repair]** On controller 2.1.2, a 20-target request preallocated and
persisted all IDs, completed twenty successful Legacy Adds, and called Start once, but physically
ran segments 0 through 9 only. Its stopped callback reported `left_paths=10`, segment 9, and the
final TCP error remained 125.582 mm; the previous Driver waited until its 300 s timeout. This is
evidence that pre-Start Adds do not make more than ten targets continuously consumable by the
controller. `MP_Motion_API.h` does not state this capacity. API2 documents a ten-entry controller
queue and is corroborating evidence only; the Driver remains on Legacy APIs.

The executor now reserves/persists the complete PathID block before the first Add, submits an
initial ten-target window, calls `MP_StartRealTimeMotion` once, and refills only from the worker
when a newer cached Motion callback reports vacant paths via `nLeftPathNum`. It never invokes SDK
from the callback, starts again, clears/replays normal execution, changes PathIDs, or splits the
Action. Each target is added once. A fresh stopped callback with unsubmitted targets,
`left_paths>0`, or no final PathID is immediately classified as queue underflow/premature stop and
enters normal stop confirmation. Software tests cover this behavior; 20/32-target continuity still
requires HIL. No controller/Driver connection, lifecycle transition, motion, IO, or PathID state
file operation was performed for this source-only repair.

## 2026-09-11 FollowCartesianPath timeout repair

**[Source inspection and software test]** The single-point HIL reached the commanded TCP tolerance
but remained EXECUTING until its 120 s timeout. Source review confirmed that timeout handling then
reused the already-expired Action deadline for Stop confirmation, making that confirmation loop run
zero times. The executor now uses the independent positive parameter
`stop_confirmation_timeout_ms` (default 3000 ms), and requires a Motion callback sequence newer
than the snapshot taken when the Stop SDK call returns, stopped, and matching the active PathID set. Stop confirmation preserves the
original failure stage; Stop SDK failure or confirmation timeout sets `STAGE_STOP` and the recovery
lock.

Motion and HAR caches now increment sequences under their respective locks and return sequence plus
data in one snapshot. Feedback/log diagnostics expose raw `nPathID`, `nPreTargetID`,
`nNextTargetID`, stopped/`eStopFlag`, `nLeftPathNum`, callback age/freshness/path match, parsed
segment, running/terminal evidence, HAR motion status/sequence/new-sample state, stable count, TCP
errors, and a concise blocker. Timeout results include the last snapshot. Identical diagnostic
states are not repeatedly logged.

The strict Legacy final rule was not relaxed. The existing HIL record lacks the raw callback values
needed to determine why terminal evidence was absent, so physical TCP arrival is not an Action PASS.
The next controller 2.1.2 HIL must capture these new raw snapshots. PathID collision retries now
return only the final complete queue attempt in the Action result; failed blocks remain persisted
and consumed. This work is source/offline-test only: no Driver/controller connection, lifecycle
transition, motion, or IO was performed, and no user PathID state file was read or modified.

## Evidence labels

- **[Source inspection]** Directly verified in the current working tree.
- **[Software test]** Recorded software build/test result; the execution date is stated.
- **[Terminal/HIL]** Reviewed result captured in an existing HIL report.
- **[User report, raw log unavailable]** User-confirmed observation without a complete retained raw transcript.
- **[Derived]** Value calculated from recorded samples, not an independent physical measurement.
- **[Pending]** Not implemented or not verified to the claimed level.

## Repository identity and working tree

**[Source inspection, 2026-09-09]**

```text
repository: /home/ai/hikrobot_ros2
branch: experiment/pathid-exhaustion-fail-safe
HEAD: f9553d7ffb49e2b2f04cf501154eb1dbb5b1767e
title: Document V0.2-03 Cartesian and visual MoveL HIL
tracked staged changes before this handoff: none
tracked unstaged changes before this handoff: none
untracked entries before this handoff: Mv3dRgbdLog/, MvSdkLog/
```

The handoff preparation itself adds this file and corrects stale lifecycle wording in
`docs/ROS_API.md` and `IMPLEMENTATION_STATUS.md`. These changes are intentionally **not committed**.
The two SDK runtime-log directories remain untracked and are excluded from the migration archive.

FollowCartesianPath development is now present as uncommitted worktree changes (Action,
Legacy MoveL queue executor, injected offline tests, and related API/architecture documents).
It has not been controller-HIL validated and has not been committed; review the current diff as
the source of truth. No real PathID state file was read or changed.

Important committed milestones in the current history:

| Commit | Committed implementation/evidence |
| --- | --- |
| `728076b` | Fail safe when the PathID soft sequence limit is exhausted |
| `7e3ed13` | StopMotion-originated Action termination is `ABORTED` |
| `4f3a292` | Publish HAR base pose as `/flange_pose` |
| `7d2b760` | Independent read-only Tool SDK HIL probe |
| `9c30134` | Read-only ToolManager and `/tcp_pose` |
| `dcd9a03` | Reviewed V0.1 through V0.2-02 HIL report |
| `2efd257` | Base-frame Active-Tool TCP contract for Cartesian Actions |
| `578dd04` | Guarded request-specific visual three-point MoveL utility |
| `f9553d7` | V0.2-03 Cartesian/visual MoveL HIL report and current manual-lifecycle launch |

No uncommitted production implementation was found at the start of this handoff. Features listed
as pending below are plans, not hidden worktree implementations.

## Environment and project layout

Reviewed baseline:

```text
OS: Ubuntu 22.04
ROS: ROS 2 Humble
MPSDK: V2.4.0 Build20251106 SVN1686294
SDK tree inspected: /home/ai/SDK/MPSDK_PUBLIC_FULL_V2.4.0_Build20251106_SVN1686294
controller used by historical HIL: 192.168.2.64:9000
robot joints used by historical HIL: 6
```

The controller address is historical/configuration data, not evidence of a current connection.
No credential value is stored in this handoff. The runtime password is read from the environment
variable named by `programmer_password_env` (default name `HIKROBOT_PROGRAMMER_PASSWORD`).

Main packages:

- `hikrobot_mpsdk_vendor`: SDK include/library target and runtime environment hook.
- `hikrobot_msgs`: robot Actions, Services, `RobotStatus`, and `ToolInfo`.
- `hikrobot_mpsdk_core`: ROS-independent handle owner, HAR, Tool, motion, PathID, and IO calls.
- `hikrobot_driver`: lifecycle node and ROS-facing validation/publication.
- `hikrobot_bringup`: source YAML and launch file.
- `mv3d_rgbd_ros2` and `mv3d_rgbd_ros2_interface`: RGB-D camera integration.
- `tools/hil`: read-only Tool probe and request-specific guarded MoveL utility.

## Reading order and source-of-truth priority

Use this priority if statements conflict:

1. Current checked-out source, interface files, launch file, and YAML.
2. Current Git identity/status and the exact commit under test.
3. Official SDK headers and the matching SDK Demo source.
4. Reviewed reports under `docs/hil/`, with their evidence labels and dates.
5. `docs/ROS_API.md`, `docs/ARCHITECTURE.md`, and `IMPLEMENTATION_STATUS.md`.
6. This handoff and historical conversation summaries.

Start with `README.md`, `docs/ROS_API.md`, `docs/ARCHITECTURE.md`, `docs/hil/README.md`, and the
two reports indexed there. Do not use the deleted obsolete project handoff document; its removal is
recorded by `69fc9b2`.

## Launch and manual lifecycle

**[Source inspection]** `src/hikrobot_bringup/launch/hikrobot_driver.launch.py` creates one
`LifecycleNode` named `/hikrobot_driver` with `namespace=''`. The explicit namespace fixes the
historical launch-construction error caused by the missing `namespace` argument. This is a static
source finding, not a new launch/HIL result.

The launch does **not** configure, activate, request control, enter AUTO, or turn motors on. Its
default YAML is the source-tree file:

```text
~/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml
```

Override it explicitly when needed:

```bash
ros2 launch hikrobot_bringup hikrobot_driver.launch.py \
  config_file:=/absolute/path/to/hikrobot_driver.yaml
```

The intended operator sequence is manual. The following commands are documentation only and were
not run while producing this handoff:

```bash
# Terminal 1: creates an UNCONFIGURED lifecycle node only
ros2 launch hikrobot_bringup hikrobot_driver.launch.py

# Terminal 2
ros2 lifecycle get /hikrobot_driver
ros2 lifecycle set /hikrobot_driver configure   # UNCONFIGURED -> INACTIVE
ros2 lifecycle set /hikrobot_driver activate    # INACTIVE -> ACTIVE
ros2 lifecycle set /hikrobot_driver deactivate  # ACTIVE -> INACTIVE
ros2 lifecycle set /hikrobot_driver cleanup     # INACTIVE -> UNCONFIGURED
```

`configure` creates the sole SDK handle, connects, starts HAR, waits for valid state, and loads the
active Tool. `activate` is state-changing: it requests control, uses PROGRAMMER/AUTO, MotorOn, and
enters realtime mode. Before activation, teach-pendant control must be released; a historical first
failure was `MP_RequestControlAccess failed: 0x82178005`, and no broader protocol meaning is claimed.
`deactivate` stops/tears down realtime operation, powers off, restores mode, and releases control.
`cleanup` disconnects and destroys the handle. Do not issue a transition from an inapplicable state.

## Current ROS contract

### Actions

| Name | Goal contract | Feedback units/meaning |
| --- | --- | --- |
| `/hikrobot_driver/move_j` | six named joint positions in rad; speed 1..100 | target/actual/error in rad |
| `/hikrobot_driver/move_l` | `target_pose = ^B T_TCP_target` | TCP target/actual, position error m, orientation error rad |
| `/hikrobot_driver/move_c` | via and target are `^B T_TCP` | same TCP contract |
| `/hikrobot_driver/move_s` | up to 49 via poses plus final `^B T_TCP` | same TCP contract |

All Cartesian `PoseStamped` inputs must have the exact configured `base_frame` (`base_link` in the
current YAML). Empty, camera, flange, Tool, non-Base, non-finite, zero-quaternion, and SDK-range-
overflow inputs are rejected. Finite non-unit quaternions are normalized. There is no Driver-side
TF conversion.

Before each Cartesian execution, the Driver refreshes and snapshots the controller Active Tool via
the existing core handle and SDK mutex. It requires a flange-mounted Tool and sends legacy SDK
Base coordinate `1:0`, `nToolId = active Tool ID`, and `nWorkPieceId = -1`. The YAML `tool_id` does
not override Cartesian Actions. One request uses the same Tool snapshot for its SDK parameter,
feedback, error, and stable completion:

```text
^B T_TCP_actual = ^B T_F_actual × ^F T_Tool_snapshot
```

The default pre-Add guard is 0.05 m and 0.1745329252 rad per segment, including every MoveC/MoveS
segment. Rejection occurs before core execution, SDK Add, and PathID reservation.

### Services

| Name | Contract/status |
| --- | --- |
| `/hikrobot_driver/set_digital_output` | `(name, value)`, write requires ACTIVE; IO HIL succeeded per operator 2026-10-24; raw case details pending |
| `/hikrobot_driver/get_digital_output` | `(name) -> value`; current IO status is operator-confirmed success, no invented per-channel transcript |
| `/hikrobot_driver/get_digital_input` | `(name) -> value`; current IO status is operator-confirmed success, no invented per-channel transcript |
| `/hikrobot_driver/stop_motion` | requests `MP_StopRealTimeMotion`; not a safety-rated emergency stop |
| `/hikrobot_driver/get_active_tool` | controller read-only query; refreshes successful active-Tool cache |
| `/hikrobot_driver/get_tool_info_list` | controller read-only Tool-list query; does not switch Tools |

Every response carries `success`, the applicable SDK error code, and a message. A successful
StopMotion response with `message='stop requested'` means the SDK stop request was accepted; it is
not by itself proof of physical standstill. Standard Action Cancel finishes `CANCELED`. A normal
StopMotion Service, lifecycle deactivate, or another stop outside CancelGoal makes the affected
Action finish `ABORTED`; this prevents the historical illegal `EXECUTING -> CANCELED` transition.

### Topics and coordinate/unit definitions

| Name | Type and contract |
| --- | --- |
| `/joint_states` | `sensor_msgs/JointState`, joint positions in rad |
| `/hikrobot_driver/flange_pose` | `PoseStamped`, `^B T_F` from HAR `stBasePose`, m/normalized quaternion |
| `/hikrobot_driver/tcp_pose` | `PoseStamped`, `^B T_TCP = ^B T_F × ^F T_Tool`, m/normalized quaternion |
| `/hikrobot_driver/status` | `hikrobot_msgs/RobotStatus` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

`ToolInfo.flange_to_tool = ^F T_Tool`; SDK Tool translation/Euler input is mm/degree with ZYX Euler,
and the ROS message is m/normalized quaternion. Flange/TCP lifecycle publishers publish only while
ACTIVE and require valid, non-stale HAR state; TCP additionally requires a valid cached flange-mounted
Tool. Pose timestamps are ROS publication time, not mapped controller acquisition time. No TF is
published.

An external Tool switch is not polled at 20 Hz. Call `get_active_tool` or repeat lifecycle setup to
refresh the cache. Cartesian motion takes a per-request snapshot, but an external controller-side
Tool change can still race the refresh-to-SDK-Add interval.

## Implementation and evidence status

### V0.1 baseline

**[Terminal/HIL, reviewed in `docs/hil/V0.1_to_V0.2-02_HIL_REPORT.md`]** Configure/HAR, Activate,
MoveJ, Action Cancel, StopMotion, PathID persistence, and bounded pre-start `0x82068043` collision
recovery have PASS records. IO/DO was deferred by user decision and is **not PASS**.

Historical PathIDs in reports are evidence snapshots. They are not the current controller state or
current local allocator state. The allocator persists before SDK Add, never accepts client PathIDs,
does not retry after Start may have begun motion, and refuses to wrap at its soft limit.

### V0.2-01 and V0.2-02

**[Terminal/HIL]** HAR `stBasePose = ^B T_F`; `/flange_pose` lifecycle/static/dynamic checks passed.
The read-only probe showed `MP_GetActivateTool`, `MP_HAR_GetToolInfoList`, and
`MP_HAR_GetToolInfo` worked without requesting control. Tool 2 (`tool2`) was a flange-mounted Tool.
The ToolManager services, cache, and `^B T_TCP` composition passed the static, J6 lever-arm, and
round-trip checks recorded in the V0.1-to-V0.2-02 report.

### V0.2-03 Base Cartesian and visual MoveL

**[Terminal/HIL, reviewed report]** Empty frame, `camera_link`, and Base Z +51 mm targets were
rejected with `INVALID_GOAL`, `path_id=0`, no motion, and persisted state remaining `788857218`.
Base Z +10 mm MoveL succeeded at PathID `788857219`; return succeeded at `788857221`; the reported
state file then held `788857222`.

**[Terminal/HIL plus derived residuals]** Request-specific segmented visual-guided MoveL has a
successful record through D7: PathID `788857253`, `SUCCEEDED`, followed by historical persisted
state `788857254`. Its approximately 0.0155 mm value is TCP-to-command tracking error derived from
two messages, not independently measured hand-eye absolute accuracy. These values are historical,
not current robot position or PathID.

**[Software test, 2026-09-08]** The stored closeout report records four selected packages built,
16 core GTests and 12 driver GTests passed (28 GTest cases; colcon aggregate 30), with zero errors,
failures, or skips. The visual utility passed `py_compile`. These tests were not rerun for this
handoff-only task.

Evidence locations:

- `docs/hil/V0.1_to_V0.2-02_HIL_REPORT.md`
- `docs/hil/V0.2-03_BASE_CARTESIAN_AND_VISUAL_MOVEL_HIL_REPORT.md`
- `docs/HIL_TEST_PLAN.md`
- `tools/hil/README.md`

## Visual calibration and historical request data

The local calibration file was found at `/home/ai/Desktop/hkhand_eye_matrix89.json` and is copied
as evidence into the migration archive, not added to the repository. Its SHA-256 is
`c569e78d15dad4e542c3fe1d7d6f511263a451626c0f93bf55dbbc8bf8c2eb64`.

Historical conversion contract:

```text
camera frame: camera_rgb_optical_frame
request_id: 09403b2d-178c-4c68-8d19-f8c2c28356ce
configuration: Eye-in-Hand
matrix direction: camera -> gripper/end-effector; used in the HIL report as ^F T_C
point_flange = ^F T_C × point_camera
camera points: m
calibration-file matrix translation: mm
```

The JSON describes its output as robot gripper/end-effector. Treating that frame as the flange is
the historical test contract; the exact physical flange/gripper frame equivalence must be rechecked
before production integration.

The capture/conversion flange pose preserved in the report was:

```text
position m: (1.0897214371003359, 0.011625484502960448, 0.746488487628611)
orientation xyzw: (0.9992937063906535, -0.026987758922005837,
                   0.026064620316856493, 0.002093992391628428)
```

**[User report/committed HIL summary]** The repository does not preserve the original image or
controller acquisition timestamp for this pose, nor a complete raw terminal transcript proving
time alignment. Therefore timestamp synchronization is not complete. The old camera points, fixed
Base points, pose, and request ID are historical test inputs only and must never become defaults for
a new motion. Do not recompute old camera points using a newer flange pose or reuse the fixed Base
points after changing the workpiece, camera mount, or calibration.

`tools/hil/visual_three_point_movel.py` is committed, dry-run by default, requires explicit
`--execute` plus an interactive phrase for motion, and uses the ROS MoveL Action rather than MPSDK.
It is request-specific, stops after a failed goal, uses 50/100 mm clearances and <=40 mm segments,
and holds the initial TCP orientation. It is not a collision planner, continuous interpolator, or
`FollowCartesianPath`. Its complete `NEAR_START -> MIDDLE -> NEAR_END` automatic HIL remains pending.

## Historical limitations as of the original handoff (current status above takes precedence)

- **[Deferred]** IO/DO HIL. The ROS/core paths exist, but safe channel naming, wiring, readback,
  fail-state, lifecycle, and physical-output behavior still require an approved HIL procedure.
- **[Pending HIL]** MoveC and MoveS; current evidence is software-only.
- **[Updated 2026-10-24]** `FollowCartesianPath` is implemented but documented use is limited to
  **1..10 points** after the failed eleven-point HIL. Online visual interpolation remains separate.
  Sending several MoveL goals sequentially is stop-to-stop sequencing, not proof of continuous interpolation.
- **[Not implemented]** Dynamic camera-to-Base transformation node, acquisition-time mapping, and
  historical flange-pose lookup.
- **[Pending]** HAR stale-state fault injection and recovery verification.
- **[Pending]** External Tool modification race-window HIL and mitigation design.
- **[Pending HIL]** SDK Euler behavior near singularities and MoveL speed 3%/5%.
- **[Observed limitation]** `motion_execution_percent` remained `0.0` in historical HIL.
- **[Pending]** Tool 2 mass is zero; validate payload semantics before relying on load compensation.
- **[Pending]** No SDK Add mock currently proves call suppression; validation/order has pure-function
  tests and HIL rejection evidence instead.
- **[Pending]** Eye-in-Hand absolute accuracy needs physical metrology, multi-pose repeatability,
  and synchronized image/robot sampling.

## IO simplification and passive Health Monitor (2026-10-15, uncommitted)

The historical welding failure is now attributed to external high-frequency arc interference
restarting the controller, not to evidence that a high DO makes Legacy GetDO intrinsically unsafe.
The protected `start` state machine, `arc_output_name`, cached UNKNOWN/LOW/ASSERTED state,
high-level read suppression, `-10004`, and arc mutex were removed. GetDI/GetDO now each call
`MP_GetConfigIOLevelStatus` once, and SetDO calls `MP_SetConfigIOLevelStatus` once with no implicit
Get, retry, or cached-state comparison. All names, including `start`, share this behavior.

A passive Driver Health Monitor now samples only memory state: connection callback, periodic HAR
cache, `CommandChannelHealth`, watchdog/recovery state, and the official emergency enum. Defaults
are enabled, 1000 ms checks, and a 3000 ms HAR timeout. It runs after configure in ACTIVE and
INACTIVE, stops before cleanup, never calls the vendor SDK or waits for the SDK command mutex, and
logs only state transitions. Only `E_MP_EMERGENCY_ACTIVE` (official value 1) is treated as asserted;
emergency blocks motion without being mislabeled as disconnect. Confirmed HAR timeout while the SDK
still reports connected permanently locks command recovery, as do disconnect and command timeout.

Cleanup now checks recovery before SDK teardown and again after deactivate. A possibly in-use handle
is not unsubscribed, disconnected, or destroyed after an uncertain SDK outcome. Removing the arc
mutex also removes the known path where watchdog/disconnect handling waited behind a blocked arc IO
call.

This remains offline-only work. No controller, Driver, lifecycle transition, motion, IO, or PathID
state access was performed.

### Bounded IO executor (2026-10-16, uncommitted)

The second step is implemented as one Runtime-owned worker for only GetDI/GetDO/SetDO. Service
requests are copied into owning tasks with QUEUED/RUNNING/COMPLETED/CANCELLED states. The worker
shares the existing Handle and `sdk_mutex`; it changes QUEUED to RUNNING atomically only after the
mutex and recovery checks, immediately before the Legacy call. A queue/lock timeout cancels the task
and returns `-10001`, so it cannot execute late. A RUNNING timeout returns `-10002` with UNKNOWN
physical outcome, poisons CommandChannelHealth and the executor, cancels queued tasks, and makes new
IO requests return `-10003`. Late SDK success cannot clear recovery or restart queued work.

Defaults give an approximately 4000 ms Service budget: 1000 ms queue/mutex admission plus 3000 ms
in-SDK watchdog. Cleanup pauses submissions, cancels non-running work, and waits one watchdog period
for idleness. It never destroys a Handle still possibly used by the worker. The worker is joined on
normal destruction and is never detached. If the vendor call is permanently blocked, safe object
lifetime and bounded process exit cannot both be achieved with the SDK API: Runtime destruction will
wait. Operational recovery therefore requires an external supervisor to allow a bounded graceful
shutdown and then terminate/replace the whole process. The Driver does not automatically kill itself.

## Historical IO HIL entry procedure (IO success reported 2026-10-24)

Do not start by changing CMake or SDK discovery: the implemented path is already in
`RobotRuntime::{set_digital_output,get_digital_output,get_digital_input}` and the three ROS Services.
Review `docs/HIL_TEST_PLAN.md`, `MP_IO_API.h`, and `MP_IO_Struct.h`. Before any write, the operator
must identify an explicitly safe, disconnected/non-hazardous DO by controller documentation and
wiring review, establish expected fail state, release teach-pendant control as applicable, and agree
on an ON/read/OFF/read sequence with guaranteed OFF cleanup. Read-only DI/DO discovery should come
before a write. Do not infer a safe channel name from old logs.

## Historical planning notes: automatic continuous visual trajectory

Start from the strict Base TCP Action contract and the pure functions in
`hikrobot_driver/cartesian_contract.*` and `hikrobot_mpsdk_core/tool_transform.*`. First define an
explicit timestamp discipline: image timestamp source, controller/HAR time mapping, bounded history
of `^B T_F(t)`, interpolation policy, staleness threshold, and rejection behavior. Reconfirm the
camera optical frame and `^F T_C` physical frame definition. Then define whether the required
controller behavior is stop-to-stop waypoint execution, buffered blending, or a new continuous path
Action; do not rename sequential MoveL as `FollowCartesianPath`. Preserve Active Tool snapshots,
Base-frame validation, per-segment safety limits, Cancel/Stop semantics, and PathID ownership.

## Migration archive notes

The archive accompanying this document includes current `src`, `docs`, `tools`, root build/readme
entry points, Git metadata/diffs, selected official SDK headers and Demo source, and the local hand-eye
JSON. It excludes `.git`, `build`, `install`, `log`, SDK runtime logs, binaries, credentials, and the
live PathID state file. No current PathID value was read. Historical values above must never be used
to overwrite `~/.ros/hikrobot/path_id.state`.

Missing evidence is listed in the archive manifest/metadata. In particular, complete raw terminal
transcripts for several historical HIL runs, original `/tmp/hikrobot_movel_*` logs, the visual
request payload outside the summarized points, and a synchronized capture timestamp are unavailable;
none are reconstructed or inferred.
