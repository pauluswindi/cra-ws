# Passive simulation loop profiling

The simulation now observes actual `MjData.time` advancement against monotonic
wall time. It emits one `Simulation profile: {...}` JSON log approximately every
five wall seconds, after the existing sleep and outside the viewer lock. This
does not add catch-up steps, change the timestep, skip feedback/viewer work,
change control gains or limits, or publish a new ROS topic.

This is the next diagnostic step after the recorded joint-5 tracking replay.
An already-running Python process must be restarted to load the instrumentation.
The profiler does not identify the live slowdown until new measurements exist.

## Capture a live trial

Build from the workspace in the usual ROS/container environment:

```bash
colcon build --packages-select simulation
```

Stop the previous CRA launch, then launch once from a shell with the normal CRA
environment loaded. Preserve the existing experiment parameters:

```bash
ros2 launch cra cra.launch.py rviz:=true 2>&1 | \
  tee "/tmp/cra_sim_profile_$(date +%Y%m%d_%H%M%S).log"
```

Startup should include:

```text
Simulation profiling: enabled=True, interval=5.000s, physics_dt=0.002000s, feedback_stride=5
```

Capture roughly 10 seconds with the robot holding, then one ordinary target
move, then the near-human situation that caused repeated stops. Note when each
phase happened. Human/robot safety behavior is unchanged. Logs are needed in
addition to any bag: the profiler writes log messages, not a recorded topic.

The ROS parameters are read at construction:

- `profile_simulation`: boolean, default `true`.
- `simulation_profile_interval`: positive finite seconds, default `5.0`.

These initial values can be overridden with normal ROS node arguments. There is
no dynamic reconfiguration callback; `ros2 param set` is not a supported way to
change the active profiler. The existing CRA launch uses the defaults.

## Read the output

`real_time_factor = physics_advanced_s / wall_elapsed_s`:

- Approximately 1: the physics clock keeps pace with wall time.
- Approximately 0.5: one wall second advances approximately half a physics second.
- Above 1: physics advanced faster than wall time within the observation window.

The ratio is measured directly, rather than inferred from feedback counts. Each
window also reports the physics start/end, fixed timestep, loop count, step rate,
feedback rate, work mean/max and the count of work iterations exceeding the
timestep budget. `loop_mean_ms` includes measured work, sleep and measurement
overhead inside an iteration; it excludes the subsequent profile-output cost.
`wall_elapsed_s` and the ratio include that cost in the next window.

Stage totals are exclusive and can be compared within the same window:

| Stage | What is measured |
| --- | --- |
| `ros_callbacks` | Existing nonblocking `rclpy.spin_once`, including the callback it dispatches |
| `viewer_lock_wait` | Acquiring the existing viewer lock |
| `control` | Existing `update_control`, including its occasional debug log |
| `physics` | Existing single `mujoco.mj_step` |
| `feedback_and_markers` | Existing feedback/visualization publication every five steps |
| `viewer_sync` | Existing viewer synchronization every five steps |
| `sleep` | Actual existing sleep duration, including scheduler oversleep |
| `profile_output` | Serialization/logging of the previous summary, counted in the next window |

Each stage has count, total/mean/max milliseconds and its share of the entire
wall window. `requested_sleep_total_ms` distinguishes intended sleep from actual
sleep. `unattributed_wall_ms` keeps loop bookkeeping, profiler/report construction,
lock release and other gaps visible. The parent loop/work statistics are inclusive;
do not add them to the leaf-stage totals.

For example, a large `viewer_sync.wall_percent` identifies time spent in viewer
synchronization within the observed process. It does not by itself establish a
GPU, rendering or mutex root cause. Large callback/publication times call for
different follow-up measurements. High sleep overshoot with little busy work
instead suggests scheduling delays. A backwards physics-time delta produces a
null ratio and a reset flag; physics reset detection is limited to the window
end being earlier than its start.

The accumulator stores only count/sum/max for eight fixed stages; memory does
not grow with the number of iterations. Output is rate-limited, but all timing
instrumentation has observer overhead. This measurement does not modify how
Motion interprets its own clock, and does not fix Vision validity or A* expiry.

## Verification and scope

```bash
PYTHONPATH=src/simulation:$PYTHONPATH python3 -m unittest discover \
  -s src/simulation/test -p test_hold_control.py
PYTHONPATH=src/simulation:$PYTHONPATH python3 -m unittest discover \
  -s src/simulation/test -p test_loop_profiling.py
```

Six existing MuJoCo hold/tracking tests and eight new profiling tests pass. The
new integration tests execute the production run loop, real physics and control
with a fake ROS spin, viewer and deterministic wall clock. Enabled and disabled
profiling produce identical control/physics order, five-step feedback cadence,
sleep requests and physical state for those controlled trials. Injecting viewer
delay lowers the measured factor and is attributed to `viewer_sync` as expected.
These synthetic timings are not live viewer measurements.

The package builds, installed source hashes match, and native ROS construction
verifies the new parameters in isolated localhost domain 223 without running the
viewer/loop or publishing motion commands. Full CRA/ROS/viewer behavior and the
live stage responsible for the slowdown still require the next user trial.
