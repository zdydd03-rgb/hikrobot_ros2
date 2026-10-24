# FollowCartesianPath: first-version usage and implementation

This document describes the current implementation of
`/hikrobot_driver/follow_cartesian_path`. It is intended for preplanned straight-line
MoveL queueing; it is not a production planner or the Motion API2 `FollowCartesianPath`.

## Current usage boundary — 2026-10-24

**Use 1..10 targets only in this version. The 11-point HIL test failed**, confirmed by the
operator on 2026-10-24. More than ten points are excluded from documented usage. IO HIL
succeeded on the same date; that outcome does not validate the motion queue.

This restriction does not assert that every 1..10-point case has an independently reviewed PASS.
No complete log or test-build revision was supplied for the failed test; do not infer its cause
from the historical ten-entry-window observation. See [current evidence](hil/V1_STATUS_2026-10-24.md).

**Runtime/source distinction:** `kMaxFollowCartesianPathTargets` is still 32 and the Action
still accepts up to that implementation bound. This documentation-only update does not add a
runtime ten-point rejection. Applications must reject goals with more than ten points. A request
accepted by the current code is not automatically within this version's supported usage.

## Contract

For current documented use, the goal contains **1..10** ordered `geometry_msgs/PoseStamped` poses, `speed_percent`, integer
`blend_radius` (0..10), and an optional timeout. Every pose must use the configured
`base_frame` (`base_link` in the current configuration) and represents the controller Active
Tool TCP (`^B T_TCP`). No TF conversion, camera transform, tool switching, or online correction
is performed. ROS metres/quaternions are converted to SDK millimetres/ZYX Euler degrees.

## Legacy SDK sequence

Using RobotRuntime's single handle and SDK mutex, the executor validates state and one immutable
Active Tool snapshot, reserves and persists two PathIDs per target, then calls:

1. `MP_ClearMotionTarget`;
2. `MP_AddMovlTarget` for the initial controller window only (at most 10 targets,
   each with `nCsType=1`, `nCsID=0`, `nToolId=active`, `nWorkPieceId=-1`);
3. `MP_StartRealTimeMotion` once;
4. from the execution worker, use each new cached Legacy Motion callback's
   `nLeftPathNum` to refill vacant window slots with remaining targets.

Controller 2.1.2 HIL observed a ten-target active Legacy queue window: a 20-target request
accepted all pre-Start Adds but ran only targets 0 through 9. The Legacy header does not document
that maximum; API2's documented ten-entry controller queue is corroborating evidence only, not
this Driver's calling interface. The implementation calls Start once and
never clears, restarts, or splits normal execution. Every target retains its preallocated PathID
and is added once. For the current 1..10-point usage boundary, no post-Start refill is needed;
the larger-path refill mechanism remains implemented but is outside the documented usage scope.

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
final path and report stopped with zero remaining paths. Final-target evidence may either
reference the final target through `nPreTargetID` or `nNextTargetID`, or use the implemented
controller-2.1.2 zero/zero target sentinel, but the sentinel requires an earlier matching running
callback and a terminal callback newer than the latest running evidence. TCP tolerance or HAR stopped status alone cannot complete the
Action. The September 2026 single-point HIL physically reached its target but timed out because the
retained evidence did not include the raw Legacy callback fields needed to explain which strict
condition remained false. The current zero-target compatibility branch is source/offline evidence,
not proof that the failed 11-point test passed or that its cause is known.

`stop_confirmation_timeout_ms` is independent of the Action timeout and defaults to 3000 ms. A
timeout/cancel/state failure snapshots the Motion callback sequence and request time **before**
requesting `MP_StopRealTimeMotion`, then waits within that separate request-time deadline for a newer stopped
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

Success additionally requires every target to have been submitted. After Start, a fresh stopped
callback with unsubmitted targets, positive `nLeftPathNum`, or no final PathID observation is a
queue-underflow/premature-stop failure, never success. It enters the independent stop-confirmation
path and reports callback sequence, path/segment, left paths, submitted/total, next target, last
submitted PathID, running evidence, and HAR motion status. Refill Add failures (including
`0x82068043`) do not reserve IDs, retry the path, or start again.

## Offline verification

`test_legacy_movel_queue_executor` injects a fake Legacy backend, fake clock, and temporary PathID
state file. It covers validation and bounds, persistence-before-Add, unique IDs, one Start for N
Adds, collision retry, partial cleanup and recovery lock, cancellation/Stop races, stale HAR,
independent stop confirmation timing, old/wrong/matching stop callbacks, callback/HAR sequences,
completion blockers, conservative completion and final success. It additionally scripts 1, 10, 20
and 32 target queue windows, repeated/out-of-order callbacks, refill failures/collision,
refill cancellation, and premature stopped callbacks. No real SDK connection or user
PathID state is accessed. The companion Cartesian tests cover frame/value conversion and step
guards.

## Current HIL outcome and remaining work

The 2026-10-24 **11-point queue test failed**. Keep first-version documented use at **1..10 points**.
The historical 20-point failure and the FakeBackend 20/32-target tests describe background and
implementation coverage only; they do not expand this version's usage limit. Raw intermediate/final
callback semantics, actual failure stage, blend behavior and timeout/stop confirmation need retained
HIL evidence. Diagnose the eleven-point failure before any proposal to expand the documented limit.
The next HIL must retain the raw diagnostic snapshots rather than infer callback state from physical
arrival. MoveC/MoveS HIL, online visual correction, hand-eye integration and welding IO are outside
this change.

Evidence sources: `MP_Motion_API.h`, `MP_Motion_Struct.h`, `MP_Public_Defines.h`, and the official
`EnterRealTimeMotionMode` legacy demo in the installed MPSDK V2.4.0 Build20251106 package.
