# FollowCartesianPath design and offline verification

This document describes the first software-only implementation of
`/hikrobot_driver/follow_cartesian_path`. It is intended for preplanned straight-line
MoveL queueing; it is not a production planner or the Motion API2 `FollowCartesianPath`.

## Contract

The goal contains 1..32 ordered `geometry_msgs/PoseStamped` poses, `speed_percent`, integer
`blend_radius` (0..10), and an optional timeout. Every pose must use the configured
`base_frame` (`base_link` in the current configuration) and represents the controller Active
Tool TCP (`^B T_TCP`). No TF conversion, camera transform, tool switching, or online correction
is performed. ROS metres/quaternions are converted to SDK millimetres/ZYX Euler degrees.

## Legacy SDK sequence

Using RobotRuntime's single handle and SDK mutex, the executor validates state and one immutable
Active Tool snapshot, reserves and persists two PathIDs per target, then calls:

1. `MP_ClearMotionTarget`;
2. `MP_AddMovlTarget` once for each target (`nCsType=1`, `nCsID=0`, `nToolId=active`,
   `nWorkPieceId=-1`);
3. `MP_StartRealTimeMotion` once.

The exact pre-Start `0x82068043` collision may retry the complete queue with a new ID block.
After Start is attempted there is no automatic retry. Partial Add failures clear the queue;
failed or uncertain cleanup requires lifecycle recovery. `nBlendRadius` is the SDK integer
0..10 field; the SDK header defines 0 as its preset blend behavior and this project makes no
precision-stop or constant-speed claim.

## Cancellation and completion

Cancel/Stop/lifecycle requests are checked before and between Adds and around Start. Start and
Stop are synchronized without holding the SDK mutex during monitoring. Completion requires a
post-Start matching queue callback, terminal stopped evidence referencing the final target,
TCP tolerance, and stable samples from distinct HAR records. Unknown progress is reported as
unknown; `dPercent` is not converted into a fabricated percentage.

The controller 2.1.2 completion rule remains deliberately strict: the callback must identify the
final path, report stopped with zero remaining paths, and reference the final target through
`nPreTargetID` or `nNextTargetID`. TCP tolerance or HAR stopped status alone cannot complete the
Action. The September 2026 single-point HIL physically reached its target but timed out because the
retained evidence did not include the raw Legacy callback fields needed to explain which strict
condition remained false. There is currently no evidence to change that compatibility rule.

`stop_confirmation_timeout_ms` is independent of the Action timeout and defaults to 3000 ms. A
timeout/cancel/state failure requests `MP_StopRealTimeMotion`, snapshots the Motion callback
sequence when that SDK call returns, and waits up to this separate deadline for a newer stopped
callback whose PathID belongs to the active queue. An old cached callback or a fresh callback for a
different PathID cannot confirm the stop. A confirmed stop preserves the original failure stage and
does not set the recovery lock. Stop SDK failure or confirmation timeout uses `STAGE_STOP`, retains
the SDK error when applicable, and requires lifecycle cleanup before another motion.

Every Motion and HAR callback increments a sequence while holding its cache lock, and snapshots
return the data and sequence together. Stable counting uses distinct post-Start HAR sequence values,
not timestamp equality or executor polling count. FollowCartesianPath feedback exposes raw Motion
callback fields, callback age and path/segment matching, running/terminal evidence, HAR sequence and
new-sample status, stable count, errors, and a concise `completion_blocker`. Identical completion
diagnostics are not repeatedly logged. Timeout/error result messages retain the last concise
diagnostic snapshot for HIL capture.

On bounded PathID collision retry, failed blocks remain consumed in the persisted allocator and are
never reused. The Action result `path_ids` contains only the complete final queue attempt, not IDs
from failed collision attempts.

## Offline verification

`test_legacy_movel_queue_executor` injects a fake Legacy backend, fake clock, and temporary PathID
state file. It covers validation and bounds, persistence-before-Add, unique IDs, one Start for N
Adds, collision retry, partial cleanup and recovery lock, cancellation/Stop races, stale HAR,
independent stop confirmation timing, old/wrong/matching stop callbacks, callback/HAR sequences,
completion blockers, conservative completion and final success. No real SDK connection or user
PathID state is accessed. The companion Cartesian tests cover frame/value conversion and step
guards.

## HIL still required

This repair has source and offline-test evidence only; it has not been rerun on the controller.
Controller 2.1.2 behavior for multi-point Legacy queue continuity, raw intermediate/final callback
semantics, blend-radius behavior, timeout/stop confirmation, and queue capacity remains unverified.
The next HIL must retain the raw diagnostic snapshots rather than infer callback state from physical
arrival. MoveC/MoveS HIL, online visual correction, hand-eye integration and welding IO are outside
this change.

Evidence sources: `MP_Motion_API.h`, `MP_Motion_Struct.h`, `MP_Public_Defines.h`, and the official
`EnterRealTimeMotionMode` legacy demo in the installed MPSDK V2.4.0 Build20251106 package.
