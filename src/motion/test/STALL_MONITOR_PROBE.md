# Offline stall-detector witness

The latest 07:49:20 CRA run ends with `MOTION_ERROR/CONTROL_ERROR/STALLED`.
Its three fault lines do not identify the actual stall-triggering joint:
MotionComponent currently prints the joint with the largest tracking error.
In particular, the printed joint 1 is visibly moving in the nearby simulation
logs. All-joint feedback/commands were not recorded in a new bag for this run.

The unchanged ExecutionMonitor checks each joint every 2 s. It declares a stall
when integrated absolute commanded motion exceeds 0.2 rad, net measured
displacement from the window start is below 0.02 rad, and current reference
error exceeds 0.02 rad. The separate tracking guard uses 0.2 rad for 0.5 s.
Net displacement is not travelled distance: a joint may reverse and return
near its initial angle despite having moved substantially.

`probe_stall_monitor.cpp` passes synthetic reference/feedback/command arrays
directly to the production ExecutionMonitor. It does not use ROS, a controller,
MuJoCo, a recorded trajectory or a real robot. Its three controls produce:

| Synthetic case, 2 s window | Actual travel | Net displacement | Monitor |
| --- | --- | --- | --- |
| Moving then reversing | 0.24 rad | Approximately zero | STALLED |
| Truly stationary with requested movement | Zero | Zero | STALLED |
| Monotonic movement | 0.30 rad | 0.30 rad | NORMAL |

The reversing case has maximum tracking error approximately 0.028 rad, below
the tracking fault limit, and final error approximately 0.027 rad. It verifies
a possible false-positive class in the stall criterion. It does not prove that
the three latest live faults have this cause or authorize changing the guard.
The driver/controller/plant and actual trigger joint still need observation.

Reproduce from the workspace in the existing CRA container:

```bash
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic \
  -Isrc/motion/include -I/usr/include/eigen3 \
  -I/usr/local/lib/python3.10/dist-packages/mujoco/include \
  src/motion/test/probe_stall_monitor.cpp \
  src/motion/src/controllers/execution_monitor.cpp \
  -o /tmp/cra_probe_stall_monitor
/tmp/cra_probe_stall_monitor
```

Results are saved in test/diagnostics/stall_monitor_probe_20261117.json. The
probe is not added to Motion's production CMake targets. Production Motion,
stall/tracking thresholds and fault recovery are unchanged.

For a new live reproduction, start recording before the motion command and
include the trajectory, all joint position/velocity feedback and velocity command
alongside execution/failure status. In a second terminal with the normal CRA
environment, use a new output path:

```bash
ros2 bag record -o "/tmp/cra_stall_5cm_$(date +%Y%m%d_%H%M%S)" \
  /joint_states /motion/joint_path /motion/joint_velocity_command \
  /motion/execution_status /motion/failure_reason \
  /vision/human/geometry /simulation/target_marker /simulation/obstacles \
  /planning/status /planning/joint_path_preview
```

This permits reconstructing each joint's command integral, net displacement,
travel and reference error over the stall window. Before choosing a replacement
criterion, distinguish physical immobility, deliberate trajectory reversals,
tracking lag, oscillation and feedback noise. Simply counting travelled distance
can let oscillation look like useful progress; changing the guard needs its own
discussion and controls.
