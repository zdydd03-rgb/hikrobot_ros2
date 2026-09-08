# HIL utilities

`tool_readonly_probe/` contains the independent read-only controller Tool API probe.

`visual_three_point_movel.py` is a request-specific HIL utility for visual request
`09403b2d-178c-4c68-8d19-f8c2c28356ce`. It contains fixed Base-frame points derived offline from
that request and its capture pose. They must not be reused after the workpiece, camera mount,
calibration, or capture pose changes.

The utility uses the ROS 2 MoveL Action and never connects to MPSDK directly. It is dry-run by
default; motion requires both `--execute` and the interactive phrase `EXECUTE THREE POINTS`.
Defaults keep Tool TCP 50 mm above the seam, use a 100 mm travel clearance, retain the initial TCP
orientation, and split the route into at most 40 mm planned segments. Any rejected or failed goal
stops the remaining sequence. `--speed-percent` selects the MoveL speed.

This utility is not `FollowCartesianPath`, a production visual transform node, a collision
planner, or evidence that the complete three-point route passed HIL. See the V0.2-03 HIL report
for the verified subset and limitations.
