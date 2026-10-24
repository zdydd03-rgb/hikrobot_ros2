# Implementation status — first driver build

Updated 2026-10-24. Production code baseline: `ae33fb7d8fa4de0026effaa20a8a9ac9b2dc1ce1`.
This update changes documentation only. Exact HIL binary identity and raw logs were not supplied;
the date and new hardware outcomes below are operator-confirmed, not reconstructed transcripts.

## Current hardware status

- **IO HIL: PASS, operator-confirmed, 2026-10-24.** Detailed DI/DO cases, channel names,
  wiring, command/results and test-build revision remain to be attached. This does not establish
  PASS for watchdog, disconnect, permanently blocked SDK calls or every recovery scenario.
- **11-point Legacy MoveL queue HIL: FAIL, operator-confirmed, 2026-10-24.** Failure stage,
  final PathID, controller queue count, raw callbacks and final TCP error were not provided.
  No particular root cause is asserted.
- **Documented first-version queue use: 1..10 points only.** More than ten points are outside
  this version's supported usage. Source validation still accepts up to 32; no ten-point runtime
  gate was implemented in this documentation update. Clients must enforce the usage limit.
  The scope restriction is not an independently evidenced PASS for all 1..10-point requests.
- Existing reviewed reports establish historical Lifecycle, MoveJ, Cancel/StopMotion, PathID,
  Tool/TCP and Base TCP MoveL results for their recorded versions. MoveC/MoveS HIL is not newly
  established by this status update.

Full evidence boundaries: [2026-10-24 status](docs/hil/V1_STATUS_2026-10-24.md).

## Implemented in the current source

- Eight-package workspace: six robot-related packages and two independent RGB-D camera packages.
- ROS-independent `RobotRuntime`, one Legacy SDK handle and shared SDK command mutex.
- Manual Lifecycle Configure/Activate/Deactivate/Cleanup; launch does not automatically power on.
- Legacy MoveJ/MoveL/MoveC/MoveS Actions and cancellation/StopMotion; no automatic replay after Start.
- Base-frame Active-Tool TCP Cartesian contract; ROS rad/m/quaternion to SDK deg/mm/ZYX Euler.
- Read-only Tool services, HAR-derived joints/flange and composed TCP publication.
- Persistent PathID allocation before Add, bounded pre-Start `0x82068043` collision retry,
  fail-closed sequence exhaustion without wrapping. Keep the live state file intact.
- FollowCartesianPath Legacy MoveL queue implementation. Internal validation capacity is 32,
  but current documented usage is **1..10**. Initial window is at most ten; the implemented
  refill path does not establish successful >10-point HIL and must not be advertised as usable.
- Callback/HAR sequence diagnostics, distinct HAR completion samples and independent 3000 ms
  FollowCartesianPath stop-confirmation window.
- Uniform Legacy DI/DO dispatch through one managed serial IO executor; no protected-arc state,
  high-output Get suppression, implicit Set readback or automatic Set retry.
- Passive Health Monitor in ACTIVE and configured INACTIVE, using callbacks/cache and command
  health; no additional SDK calls or handle. Default check 1000 ms, communication timeout 3000 ms.

## Offline evidence inspected

The retained effective CTest configuration registers five core and two Driver targets.
Their 2026-10-16 XML records contain **97 GTest cases, zero failures/errors**. These existing
records were inspected, not rerun on 2026-10-24. Old `test_protected_arc_output` build artifacts
are not current registered tests. There is no complete RobotRuntime/ROS Service integration test.
The package manifests parse and the 79-file source/config/interface manifest from the 2026-10-16
handoff still matches the current tracked code. This does not prove the exact revision used on hardware.

## Limitations and remaining work

- Stop at ten points in application input. **11-point HIL failed**; larger queue/worker refill
  continuity is excluded from the current usage contract. Do not split a long queue and present
  sequential Actions as verified continuous interpolation.
- Existing 20/32-point FakeBackend test coverage is software evidence only, not supported HIL scope.
- Vendor SDK calls are synchronous and uncancellable. IO caller waits are bounded by admission and
  execution budgets plus scheduling; a permanently blocked worker can still block final destruction.
  End-to-end ROS latency and command recovery faults are not all covered by the successful IO report.
- YAML translation guard is **0.600 m**, while Node default is 0.05 m and YAML comment says 50 mm;
  this inconsistency remains unchanged and must be checked before operation.
- Health/recovery fault injection, MoveC/MoveS, queue failure diagnosis, Tool-change races and
  synchronized visual transforms remain unverified unless separately supported by retained evidence.
- Local `visual_heat_seam_moves*.py` and `Bd/hkhand_eye_matrix89.json` remain untracked and are not
  part of this release/push, per operator instruction. Runtime SDK logs also remain local.

See [queue usage](docs/FOLLOW_CARTESIAN_PATH.md), [ROS API](docs/ROS_API.md),
[HIL checklist](docs/HIL_TEST_PLAN.md) and the [detailed source handoff](docs/handoff/HIKROBOT_DRIVER_FINAL_HIL_CODE_HANDOFF.md).
