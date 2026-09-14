# ROS API (V0.2)

Node name: `/hikrobot_driver`

## Actions

- `/hikrobot_driver/move_j` — six joint positions in radians + `speed_percent` (1..100)
- `/hikrobot_driver/move_l` — target `^B T_TCP` `PoseStamped` + `speed_percent`
- `/hikrobot_driver/move_c` — via and target `^B T_TCP` poses + `speed_percent`
- `/hikrobot_driver/move_s` — up to 49 via `^B T_TCP` poses plus final `^B T_TCP` + `speed_percent`
- `/hikrobot_driver/follow_cartesian_path` — 1..32 ordered Base TCP poses, queued as one
  legacy MoveL sequence and started once. Goal fields are `poses`, `speed_percent`,
  `blend_radius` (SDK integer 0..10), and optional `timeout` (`0` keeps the driver default).
  All points are validated before PathID reservation. Feedback reports queueing/execution stage,
  queued count, current segment, TCP sample, raw Legacy callback fields, HAR/callback sequences,
  queue evidence, stable count and `completion_blocker`; result reports the result code, final
  queue-attempt PathIDs, failure stage/target and elapsed time. This is preplanned straight-line queueing, not
  FollowCartesianPath API2, online correction, or a guarantee of exact intermediate passage.

`blend_radius` maps directly to the public SDK's integer range 0..10. `timeout=0` uses the driver default.

Action clients never provide `nPathID`; the driver owns and persists it.

All MoveL/MoveC/MoveS poses must use the exact configured `base_frame` (currently
`base_link`). Empty or different frames are rejected; the Driver does not perform TF
conversion. Each pose means the controller Active Tool TCP in Base coordinates, never a
flange, Tool 0, camera, or arbitrary-frame pose. The Driver refreshes the controller Active
Tool before each Cartesian request, requires a flange-mounted Tool, and sends Base coordinate
`1:0`, that Tool ID, and no workpiece (`-1`) to the legacy realtime SDK. The retained YAML
`tool_id` does not select the Tool for Base Cartesian Actions.

Cartesian feedback `target_pose` and `actual_pose` are both TCP poses. Actual TCP and completion
errors are calculated from one per-motion Tool snapshot as `^B T_F × ^F T_Tool`; position error
is reported in metres and orientation error in radians. Input ROS m/quaternion is converted to
SDK mm/ZYX Euler degree. The default first-HIL guard rejects any current-to-first or adjacent
path segment over `max_cartesian_translation_step_m=0.05` or
`max_cartesian_rotation_step_rad=0.1745329252` (10 degrees), before PathID allocation or Add.

## Topics

- `/joint_states` (`sensor_msgs/JointState`), radians
- `/hikrobot_driver/flange_pose` (`geometry_msgs/PoseStamped`): `^B T_F`, the flange pose
  in `base_frame` (currently `base_link`). It is sourced from HAR `stBasePose`, with mm→m
  translation and degree Roll/Pitch/Yaw→quaternion orientation conversion. It is not the
  current Tool TCP pose. The driver publishes it only for valid, non-stale HAR state with at
  least six joints. V0.2-01 timestamps it with the ROS publish time; mapping the original
  HAR/controller sample time is future timestamp-discipline work.
- `/hikrobot_driver/tcp_pose` (`geometry_msgs/PoseStamped`): `^B T_TCP = ^B T_F × ^F T_Tool`.
  It is published only for valid non-stale HAR state and a cached flange-mounted active Tool.

`ToolInfo.flange_to_tool` is `^F T_Tool`. Tool data comes from controller read-only APIs;
SDK mm/degree ZYX Euler values become ROS m/normalized quaternion poses. The Driver exposes
`/hikrobot_driver/get_active_tool` and `/hikrobot_driver/get_tool_info_list`; neither writes or
switches Tools. After an external Tool switch, call `get_active_tool` or repeat lifecycle setup.
No TF is published, and camera/hand-eye transforms are not part of V0.2-03.
- `/hikrobot_driver/status` (`hikrobot_msgs/RobotStatus`)
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

## Services

- `/hikrobot_driver/set_digital_output`
- `/hikrobot_driver/get_digital_output`
- `/hikrobot_driver/get_digital_input`

`arc_output_name` (default `start`) identifies the protected welding arc DO. Its
sequence is `pre-read low -> set high -> no Get while high -> set low` (an
optional post-low read is operator-controlled). While asserted, Get returns a
local failure with the cached commanded value and never performs hardware
readback; this is not physical confirmation. SDK synchronous calls remain
watchdog-monitored but cannot be cancelled, so hardware isolation and a
physical shutoff remain required.
- `/hikrobot_driver/stop_motion`
- `/hikrobot_driver/get_active_tool`
- `/hikrobot_driver/get_tool_info_list`

`stop_motion` is a normal software motion stop. It is **not** a safety-rated emergency stop.

FollowCartesianPath uses `MP_ClearMotionTarget`, one `MP_AddMovlTarget` per point, and one
`MP_StartRealTimeMotion`. A complete `2 * point_count` PathID block is persisted before the first
Add; IDs are not supplied by clients. Only a pre-Start `0x82068043` collision may trigger a
bounded whole-queue retry with a fresh block. Once Start is attempted, no automatic retry occurs.
Cancellation and lifecycle/service stops use the existing CANCELED versus ABORTED Action-state
mapping. A failed cleanup or uncertain stop places the runtime in recovery-required state until
lifecycle cleanup.

`stop_confirmation_timeout_ms` defaults to 3000 ms and is a positive, bounded deadline independent
of the Action timeout. After a post-Start timeout/cancel/failure, Stop success must be confirmed by
a Motion callback newer than the snapshot taken when the Stop SDK call returned, stopped, and
matching the active PathID set. Confirmation preserves the original result code and failure stage. Stop SDK failure or stop
confirmation timeout reports `STAGE_STOP` and sets the recovery lock. Result messages include the
last compact completion snapshot. Failed PathID-collision attempts remain persisted/consumed but
are omitted from `result.path_ids`, which describes only the complete final queue attempt.

The strict Legacy final-completion rule remains final PathID + stopped + `nLeftPathNum == 0` + final
target referenced by `nPreTargetID` or `nNextTargetID`, followed by distinct in-tolerance stopped
HAR samples. Neither TCP arrival nor HAR stopped status alone is success, and controller 2.1.2
`dPercent` is not used. This rule awaits raw callback evidence from a new HIL run.

## Lifecycle

The bringup launch creates the lifecycle node in the `unconfigured` state and does not configure,
activate, or power on the robot automatically. Its default `config_file` is the source-tree YAML at
`~/hikrobot_ros2/src/hikrobot_bringup/config/hikrobot_driver.yaml`; override it with
`config_file:=/absolute/path/to/hikrobot_driver.yaml` when needed. Configure and activate must be
requested manually. Activation performs PROGRAMMER login, AUTO, MotorOn and realtime-mode entry.
The programmer password is read from `HIKROBOT_PROGRAMMER_PASSWORD` by default.
