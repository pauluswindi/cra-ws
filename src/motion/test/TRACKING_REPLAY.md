# Recorded tracking diagnosis

This offline experiment investigates the joint-5 `TRACKING_ERROR` recorded in
`cra_path_diag_1s_20261117_005856`. It does not start ROS nodes, publish commands,
open a viewer, or change runtime controller/model parameters.

Run from the workspace in the existing `cra-humble` environment:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/simulation:$PYTHONPATH python3 \
  src/motion/test/replay_recorded_tracking.py --verify \
  --output /tmp/cra_tracking_replay
```

Dependencies are the existing ROS C++ message headers, Eigen, g++, MuJoCo,
NumPy and PyYAML. The script compiles current production Motion sources into a
temporary shared library, rather than reimplementing the trajectory/controller
equations in Python or relying on a possibly stale installed Motion library.
The bridge follows MotionComponent's configuration/call order: trajectory speed
is `0.6 * motion.joint.speed`, then JointController, MotionGuard and
ExecutionMonitor. The existing simulation test loader stubs only ROS imports;
MuJoCo and RobotSimulation's velocity/hold/update methods are real.

The fixture includes the received 11-point trajectory, measured positions and
velocities, nearby feedback timestamps, scene markers, the logged fault and bag
database hash. All four runs use the same current YAML, XML and controller.

| Case | Input / purpose | Verified result |
| --- | --- | --- |
| Recorded feedback | Feed measured angles to native Motion at 200 Hz, holding the latest feedback | Joint-5 tracking fault at 2.245 s; live log approximately 2.248 s |
| Physics real time | Advance physics at the same rate as the controller's wall clock | Completes at 13.860 s, maximum joint error 0.0307 rad, zero contacts |
| Physics half speed | Advance 0.5 physics seconds per controller wall second | Joint-5 tracking fault at 3.245 s, zero contacts |
| Recorded cadence | Interpolate five physics steps between recorded feedback stamps | Joint-5 tracking fault at 2.585 s, zero contacts |

`diagnosis.json` records seven comparison checks, configuration, source hashes,
timing estimates, fault details and limitations. Each case CSV contains reference,
feedback, raw/guarded velocity and errors; physics runs also include actuator
targets, joint velocities, actuator/bias forces, contacts and physics time.
The `--verify` checks describe this fixture's expected causal comparison; they
are not universal success criteria for arbitrary trajectories/configurations.

## Interpretation

The production simulation loop advances exactly one 0.002 s physics step per
iteration and emits feedback every five steps. During the recorded execution,
106 feedback intervals span 2.2229 wall seconds: approximately 1.06 physics
seconds, a ratio of 0.477. Independently, changes in angle divided by reported
physics velocity imply a median 0.010002 physics seconds per feedback interval,
consistent with five steps rather than extensive message loss. No `/clock` or
direct physics-time telemetry was recorded, so these are inferred timing values.

Motion's trajectory uses `steady_clock` elapsed time, with a 5 ms wall timer.
RobotSimulation integrates the velocity command using the fixed physics step.
For a reference of 0.3 rad per wall second and physics/wall ratio 0.477, the
0.5 rad per physics second command cap yields approximately 0.2385 rad per wall
second. Sustained tracking error can grow even at the command cap. Raising gain
does not remove this rate mismatch.

This comparison shows that slow physics is sufficient to produce this class of
fault with unchanged guards, and the measured-feedback replay closely matches
the recorded joint-5 fault. It does not establish that all six faults in the
longer launch share one cause, or identify which live loop stage causes slowdown.

## Limits and next measurement

- Exact ROS delivery schedules, full initial MuJoCo state and steady-clock start
  were not recorded. The acceptance log timestamp approximates trajectory start;
  actuator targets begin in a gravity-balanced hold.
- Recorded-cadence physics steps are interpolated, including timestamps after
  the live fault. Its exact fault time is not a reconstruction of the live plant.
- The native wrapper omits ROS transport/feedback validation; all replay feedback
  ages are checked to remain below the configured 0.2 s timeout. Tracking, stall
  and joint speed/limit guards run unchanged. Physical collision contacts remain
  enabled. Planner/human collision validation is outside this isolated experiment.
- The target position is restored; target orientation and parked obstacles use
  their XML defaults. All physical replay runs report zero contacts.
- A real-time success here is not end-to-end ROS/viewer validation or permission
  to loosen Motion guards, human margins or freshness checks.

Next, measure physics-time/wall-time ratio and separate timings for ROS callback
processing, control, physics, feedback publication and viewer synchronization in
the live simulation. Then discuss a bounded simulation scheduling correction or
decoupling viewer work. Merely publishing `/clock` is insufficient while Motion
continues to use `steady_clock`, and changing all clocks would be a larger
architectural decision. Vision invalid frames and planner expiry remain separate
issues from this Motion diagnosis.
