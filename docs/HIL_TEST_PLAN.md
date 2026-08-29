# Hardware-in-the-loop test order

The ROS 2 implementation should be validated in this order on the same controller power cycle where possible.

1. Configure lifecycle node: verify connection and `/hikrobot_driver/status` state flow.
2. Activate: verify control access, PROGRAMMER login, AUTO, MotorOn, realtime mode.
3. MoveJ: current pose -> J6 + small delta -> return. This reproduces the already-proven SDK baseline through ROS Action.
4. PathID crash recovery: execute a MoveJ, terminate the node before a second command, restart, and verify the next Add does not stick on `0x82068043`.
5. DO: set an explicitly selected safe output ON/read/OFF/read.
6. MoveL: choose a short Cartesian displacement in a verified collision-free region at low `speed_percent`.
7. MoveC: verify via pose and final target semantics at low speed.
8. MoveS: begin with two or three conservative via poses, then expand up to the SDK limit only after basic behavior is confirmed.
9. Action cancel and `/stop_motion` during each supported motion class.
10. Network interruption: verify result becomes `STATE_LOST` and no automatic motion resend occurs after Start.

MoveL/MoveC/MoveS are implemented from the public legacy API and official demo, but only become HIL-VERIFIED after these tests pass.
