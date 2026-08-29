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
- `/hikrobot_driver/tcp_pose` (`geometry_msgs/PoseStamped`), base-frame metres/quaternion
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
