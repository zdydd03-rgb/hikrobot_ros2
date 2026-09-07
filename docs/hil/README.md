# Hardware-in-the-loop reports

This directory contains reviewed hardware-in-the-loop (HIL) reports for the Hikrobot ROS 2
driver. They preserve the controller-observed behavior that establishes a development baseline
and distinguish terminal observations from calculations, conclusions, limitations, and work
that has not yet been verified.

The current report, [`V0.1_to_V0.2-02_HIL_REPORT.md`](V0.1_to_V0.2-02_HIL_REPORT.md), covers the
V0.1 gate and the V0.2-01, V0.2-02A, and V0.2-02B milestones through read-only ToolManager and
TCP-pose publication.

Each later HIL stage should record:

- the exact commit under test;
- the commands that were actually run;
- preserved key output or an explicitly labelled result summary when complete terminal output
  is unavailable;
- measured or calculated results and the calculation contract;
- the conclusion, known limitations, and unverified items.

Raw SDK logs are diagnostic inputs, not a substitute for a reviewed HIL report. Reports must
not present incomplete or unavailable terminal output as a complete original log.
