# Hardware-in-the-loop regression and first-version boundary

Updated 2026-10-24: **IO HIL PASS (operator-confirmed)**; **11-point Legacy MoveL queue HIL FAIL**.
Current documented queue use is **1..10 points only**. The source still accepts up to 32;
this documentation-only update does not add a runtime ten-point gate. Clients must enforce
<=10. See [current evidence and missing logs](hil/V1_STATUS_2026-10-24.md).

This is a future/manual checklist, not commands performed during the documentation update.
Use independently reviewed safe goals and channels; do not invent coordinates or reset PathIDs.

1. Configure: verify connection and first HAR/Tool initialization. The node is INACTIVE and
   status/joint/flange/TCP lifecycle publishers have no new output yet; use transition logs and
   read-only services rather than expecting `/hikrobot_driver/status` before Activate.
2. Activate: verify control access, PROGRAMMER login, MotorOff, AUTO verification, MotorOn,
   realtime mode and equilibrium. Then inspect status/joint/flange/TCP topics.
3. MoveJ: use a reviewed small joint change and return; record Goal/Result and actual joints.
4. PathID: retain the live file; verify sequential process restart continues the persisted high
   water mark. Fault/exhaustion tests belong on isolated state files, never the real file.
5. IO: retain the successful 2026-10-24 evidence; obtain exact DI/DO names, wiring, commands,
   readback and final state. Do not assume that success also covered disconnect/watchdog/destruction.
6. MoveL: verify a reviewed short Base Active-Tool TCP goal and return. Effective YAML translation
   guard is 0.600 m despite its 50 mm comment; Node default is 0.05 m. Confirm actual parameters.
7. MoveC/MoveS: require their own retained hardware evidence; IO success does not validate them.
8. FollowCartesianPath: restrict first-version use to **1..10 targets**. Retain matching running
   and terminal callbacks, distinct HAR samples, final TCP error, PathIDs and final Result.
   The eleven-point test failed: collect its evidence and diagnose before expanding scope.
   Historical 20/32-target fake tests are not an instruction to operate those lengths on hardware.
9. Cancel/Stop: separately check early acceptance/Add/Start/execution races and final physical stop.
   StopMotion success alone is not physical stop confirmation. Keep any failure and Recovery logs.
10. Communication/health: verify ACTIVE versus INACTIVE monitoring, HAR timeout, connection loss,
    emergency status and recovery lock in a separately controlled fault-injection plan. Do not
    guarantee STATE_LOST for every network failure: timing may instead expose SDK_ERROR or
    DRIVER_NOT_READY; preserve the actual result and do not resend motion after Start.
11. Deactivate/Cleanup: verify actual mode/motor/control/handle outcome, not merely Lifecycle SUCCESS.
    Poisoned command channels may retain the Handle and require process replacement; permanently
    blocked Vendor calls can prevent graceful destruction.

This checklist does not mark an item PASS without its evidence. For current operator-confirmed
outcomes and historical reports, use the [HIL index](hil/README.md).
