# ROS API (V0.1)

Node name: `/hikrobot_driver`

## Actions

- `/hikrobot_driver/move_j` — six joint positions in radians + `speed_percent` (1..100)
- `/hikrobot_driver/move_l` — Cartesian target `PoseStamped` + `speed_percent`
- `/hikrobot_driver/move_c` — via pose + final target pose + `speed_percent`
- `/hikrobot_driver/move_s` — up to 49 via poses + final target pose + `speed_percent`

`blend_radius` maps directly to the public SDK's integer range 0..10. `timeout=0` uses the driver default.

Action clients never provide `nPathID`; the driver owns and persists it.

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
No TF is published. Cartesian Action Tool-ID policy remains separate until Base Cartesian contract work.
- `/hikrobot_driver/status` (`hikrobot_msgs/RobotStatus`)
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)

## Services

- `/hikrobot_driver/set_digital_output`
- `/hikrobot_driver/get_digital_output`
- `/hikrobot_driver/get_digital_input`
- `/hikrobot_driver/stop_motion`

`stop_motion` is a normal software motion stop. It is **not** a safety-rated emergency stop.

## Lifecycle

The bringup launch automatically configures and activates the node. Activation performs PROGRAMMER login, AUTO, MotorOn and realtime-mode entry. The programmer password is read from `HIKROBOT_PROGRAMMER_PASSWORD` by default.
