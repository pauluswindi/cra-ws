# Cognitive Robotic Arm (CRA)
# Development Status

Last updated: 2026-11-14

## 1. Project Information

Project: Cognitive Robotic Arm (CRA)

Research focus:
Robot arm motion planning and dynamic obstacle avoidance
using GNG and A* with visual perception.

Environment:
- ROS 2 Humble
- MuJoCo
- Python and C++
- Azure Kinect
- Five-joint robotic arm

Active Planning package:
src/planning/

Legacy Planning package:
src/workspace_planning/

The legacy package is no longer used.
Some functionality has been migrated into the active Planning package.

## 2. Current Architecture

Vision
    |
    v
Human Detection and Reconstruction
    |
    v
HumanCapsuleArray
    |
    v
Planning
    |
    +-- Workspace Generation
    +-- Growing Neural Gas
    +-- Joint Graph
    +-- A*
    +-- Collision Checking
    +-- Path Validation
    |
    v
Motion
    |
    v
MuJoCo Simulation

Vision provides human geometry to Planning.

Planning uses GNG and A* for path planning
and CollisionFilter for path validation.

Motion handles trajectory execution.

MuJoCo provides the simulation environment.

## 3. Current Development Focus

Main task:
Debugging human obstacle avoidance after
integrating Vision with Planning.

Reported problems:

1. Path planning frequently fails when
   human obstacles are detected.

2. Human capsules appear excessively large.

3. Capsule radius fluctuates between frames.

4. Collision checking may reject too many
   robot configurations.

Current objective:
Improve collision geometry accuracy and
human capsule stability while preserving
the existing GNG and A* architecture.

## 4. Development Progress

### 4.1 Planning

Existing implementation:
- Workspace generation
- GNG graph generation
- Joint graph construction
- A* path search
- Joint-state collision validation
- Edge collision validation
- Path collision validation
- PlanningManager orchestration
- ROS 2 integration
- Motion execution interface

Status:
Implemented in the repository snapshot.

Current runtime behavior requires verification.

### 4.2 Vision

Existing implementation:
- Human body tracking
- Skeleton transformation
- Human segmentation
- Depth point extraction
- Voxel reconstruction
- Human capsule generation
- Human geometry publication

Main topic:
/vision/human/geometry

Message type:
msgs/msg/HumanCapsuleArray

Expected frame:
base_link

Status:
Implementation available.

Human geometry accuracy and temporal stability
require further evaluation.

### 4.3 Motion and Simulation

Existing implementation:
- Joint trajectory execution
- Joint feedback processing
- Motion monitoring
- Stop and recovery mechanisms
- MuJoCo simulation integration

Status:
Implementation available.

Trajectory tracking and end-to-end execution
require further verification.

### 4.4 CRA Target Control GUI

Status: Implemented and partially tested (headless behavior checks).
Native Tk, ROS 2 transport, and MuJoCo/Planning integration are unverified.

The Simulation package provides `ros2 run simulation target_gui`; CRA's
Simulation launch also starts it when `simulation.target_gui.enabled` is true
in `src/cra/config/robot.yaml` (currently enabled).
It displays linked XY/XZ workspace projections with separate Current Target
and Preview Target markers. Dragging, numeric edits, and RESET change only
the local preview. SET TARGET validates the preview and submits one command.
Current Target changes only when valid simulation feedback arrives.

## 5. Recent Development

### 2026-11-14 — Reproduce joint-2 tracking fault and enable bounded staging

Status: Implemented and tested with the actual MuJoCo model headlessly.
ROS 2 build, transport, live Kinect and complete live execution remain unverified.
This supersedes the pending stationary-scene diagnosis below.

New live evidence:

- Motion repeatedly reports joint2 TRACKING_ERROR: reference/actual differences
  of 0.288–0.449 rad, requested velocity 0.789–1.197 rad/s, and filtered velocity
  0.500 rad/s. The existing recovery replans once, fails again and latches
  fault=True / CONTROL_ERROR / MOTION_ERROR. A subsequent valid EMPTY frame
  does not clear that control fault. These are distinct from the earlier
  HUMAN_TOO_CLOSE / HUMAN_PATH_BLOCKED stops.
- The final measured configuration is approximately
  `[2.53958, -2.27872, 2.44299, -0.05139, 1.27958]`; all five angles are within
  joint limits. Its 0.359 m requested-target error is an interrupted execution
  error, not evidence that Motion completed at an inaccurate Cartesian goal.
- At two logged failure poses, the configured model requires 12.2103 and
  13.7016 Nm of joint2 gravity support. The previous moving target clamp of
  measured angle +/-0.12 rad, with kp=100, allowed at most 12 Nm of position
  torque before damping. Thus a positive lifting velocity command could still
  let the arm sag and exceed the existing tracking threshold without contact.

Changes:

- Simulation now centers that +/-0.12 rad correction allowance on the existing
  gravity-balanced hold target. Velocity integration, +/-0.5 rad/s command
  clipping, joint/control limits, model gains and collision behavior are
  unchanged. The absolute actuator offset can exceed 0.12 rad by the model
  gravity-support term. Motion's 0.2 rad / 0.5 s tracking guard, one recovery
  attempt and fault latch are unchanged; no automatic fault-clear was added.
- Planning's existing safe-position move previously required two failed goal
  batches, while unchanged-environment caching prevented the second batch.
  It can now try one bounded staging search after NO_GRAPH_PATH exhausts the
  configured alternative goals. It still requires a valid measured start,
  valid candidate/edge/prepared path, fresh inputs and the STOP/RELEASE protocol.
  Success returns to the original Cartesian target; invalid paths and control
  faults do not trigger staging, and unowned STOP cannot be released.
- An interrupted staging move clears its completion context. Input expiry
  cannot permanently cache a failed staging attempt; fresh input can retry.
- Target input is Cartesian XYZ in CRA base_link. A* searches joint-space
  configurations and Motion interpolates joint trajectories. A straight joint
  segment does not imply a straight end-effector path in Cartesian space.

Verification:

- All 95 tests passed: 89 Planning/geometry/visualization tests and six
  simulation control tests. The new control regressions exercise three logged
  poses and actual graph waypoints from the final pose; restoring only the
  old clamp makes all four cases fail the tracking guard. Six new Planning
  tests cover staging activation/completion, bounded retries, invalid paths,
  unowned STOP, expired input and refusal to clear a control fault.
- The dynamics comparison uses actual simulation callbacks and model physics,
  with the configured linear reference, gain 2, 0.3 rad/s reference limit and
  200 Hz command schedule sampled on the model's 2 ms physics timestep. It
  reproduces the controller equations in Python; it does not run the C++ ROS
  component. All eight comparison runs have zero contacts. From the final
  pose along graph waypoints, peak joint2 error is 0.547778 rad before and
  0.023787 rad after; the corrected run reaches the unchanged 0.02 rad final
  joint tolerance. The baseline crosses the tracking guard even though the
  offline comparison continues integrating to expose its behavior.
- Logs: `/tmp/cra_tracking_fix_planning_tests.log`,
  `/tmp/cra_tracking_fix_simulation_tests.log`,
  `/tmp/cra_tracking_control_regression_before.log`, and
  `/tmp/cra_tracking_control_comparison.json`. These are local debug artifacts.

Rebuild `simulation` and `planning`, source the overlay, then restart the stack
to load the correction and start a fresh control session. Keep all collision
and human margins unchanged. Verify the repeated joint2 fault no longer occurs
in a controlled live simulation; headless results do not establish physical
robot performance or guaranteed motion through arbitrary human geometry.

### 2026-11-14 — Faster A* validation and alternative joint goals

Status: Implemented and tested headlessly after the user confirmed EMPTY
recovery works but reported slow response and difficulty routing around humans.
Live ROS/Kinect movement and latency remain unverified in this environment.

Source findings:

- A* eagerly checked neighboring states/edges, including edges whose proposed
  cost could not improve the current route. Many tens of thousands of state
  queries could run synchronously before feedback/Vision callbacks resumed.
- Normal/recovery goal selection tried only the first valid graph configuration
  near a Cartesian target. A disconnected joint branch could fail even when
  another valid end configuration satisfied the same Cartesian bounds.
- Motion command publication followed path/graph visualization work.

Changes:

- `AStarPlanner._lazy_search()` runs A* on candidate edges and checks every
  state/edge of a candidate path with the same CollisionFilter. Failed states
  or edges are cached for that environment revision and excluded from later
  candidates. Only a fully certified path is returned. Optimistic restarts are
  bounded at 32; the original eager A* and existing local-connection fallback
  remain available, with the eager search establishing a valid fallback frontier.
  GNG data, graph edge costs and the A* heuristic are unchanged.
- Skip neighbors that cannot improve g_score before expensive collision queries.
  Both search forms stop when required human data expires. Results computed
  across expiry or revision changes cannot poison the cache with false blocked
  states/edges. Temporary start/goal connections are still revalidated per request.
  The caller's final path validation and pre-execution checks remain required.
- `PathPlanner.goal_candidates()` preserves the existing 0.04–0.12 m Cartesian
  goal window. After NO_GRAPH_PATH, `find_alternative_goal_path()` tries other
  distinct, valid joint configurations, up to the existing joint_graph
  max_candidates budget (currently four configurations including the first).
  Normal planning and the existing controlled-error recovery use it. Unsafe
  starts, invalid goals, invalid paths, expiry and revision changes do not
  trigger speculative execution or an unbounded retry loop.
- `PlanningNode.planning_cycle()` dispatches STOP/RELEASE/EXECUTE before runtime
  distance diagnostics and visualization. No threads, nodes, timer changes,
  message changes, margin reductions, or relaxed freshness rules were introduced.
  execute_path=True, simulation_only_ignore_unknown=True, Planning margin
  0.10 m and Vision extra padding 0.0 m are preserved.

Verification:

- All 87 tests passed: 83 Planning/geometry/visualization tests and four
  simulation hold tests. Nine new A* tests cover blocked edges/states, all-route
  failure, valid local fallback frontier, expiry/cache recovery, environment
  changes, temporary start changes, and agreement with independent Dijkstra
  costs across 30 randomized blocked graphs. Four new real-model tests cover
  alternative joint goals reaching Motion decisions, candidate limits, and
  refusal to retry unsafe/expired input. Test output:
  `/tmp/cra_responsiveness_all_tests.log` and
  `/tmp/cra_responsiveness_hold_tests.log`.
- Controlled cold-cache comparison using identical model, graph, start and goal:

  | Scene | Before | After | State checks before/after |
  | --- | ---: | ---: | ---: |
  | EMPTY, human checks disabled by the configured simulation policy | 3.060 s | 0.039 s | 32,784 / 386 |
  | Eight synthetic OBSERVED capsules | 15.777 s | 0.154 s | 61,247 / 517 |

  Before/after joint paths are numerically identical: 4.54267 rad for the empty
  scene and 6.29366 rad for the detour, with unchanged refined minimum clearance
  above 0.10 m. The eight-capsule workload uses one detour capsule and seven
  remote capsules; it is a controlled computation comparison, not a live
  Kinect human scene. Geometry benchmarking freezes capture time explicitly.
- A separate unfrozen-clock eight-capsule search and complete final validation
  took 0.129 s and retained fresh data within the unchanged 0.5 s deadline.
  `/tmp/cra_astar_latency_comparison.json` records both comparisons.
- A real-clock Manager probe with fresh OBSERVED geometry each simulated cycle
  follows STOPPED -> PATH_READY -> RELEASE -> IDLE -> EXECUTE. Planning takes
  0.129 s; each preparation/revalidation phase takes about 0.121 s. With explicit
  0.2 s pauses and mocked Motion acknowledgments, the replan/handshake takes
  about 0.824 s overall. This is not ROS transport or dynamics execution and
  search time must not be presented as total response latency.
  `/tmp/cra_responsive_manager_real_clock.json` records the stages.
- Direct checks of the actual planning-cycle method confirm all three command
  types publish before visualization. Syntax, Flake8 (99-character limit),
  JSON/config parsing and Git whitespace checks passed.

Rebuild `planning`, source the overlay and restart CRA. New logs report A*
candidate-path counts and alternate Cartesian-goal attempts. This improves
search responsiveness/options; current-state margin violations still require
a certified retreat, and a visibly open workspace does not prove a connected
joint-space route. Stale Vision, tracking failures, coarse graph coverage and
the existing full historical-path revalidation remain separate limitations.
No physical robot safety or guaranteed escape claim is made.

### 2026-11-14 — Diagnose stationary EMPTY scene and target offsets

Status: Source audit and offline probes completed; a periodic runtime diagnostic
was added. Exact live STOP/fault cause remains pending the current Planning and
Motion status values. Target-selection and collision behavior are unchanged.

The new attachment contains 208 lines, beginning mid-message, and covers about
42 seconds of a stationary pose. It includes 20 EMPTY, 12 STALE_MESSAGE and
10 VISION_INVALID transitions, but NO Planning status, Motion status/failure,
requested-target coordinate, accepted trajectory, or completion message. There
are no OBSERVED human reports in this excerpt. Joint commands/velocities are
zero and simulation reports contacts=0. Human diagnostic status UNKNOWN alone
does not identify the motion blocker under simulation_only_ignore_unknown=True.

Findings:

- Measured joints are approximately
  `[0.1696632851, -1.9255086910, 2.1852203227, 0.4095203659, -0.1680468340]`.
  They satisfy all joint limits and diagnose VALID against the default XML
  environment with humans disabled. The live target/static poses are absent,
  so this does not certify the actual live environment. Their end-effector
  coordinate in CRA base_link is approximately
  `[0.03896674, -0.28463422, -0.25867694]` m.
- With this measured pose, a fresh EMPTY snapshot and the default XML target,
  a fresh manager reaches PATH_READY then EXECUTE/TRAJECTORY_SENT with seven
  validated waypoints. This offline state-machine probe does not reproduce the
  live target or its unknown prior STOP/fault history; it confirms EMPTY alone
  does not block a fresh manager under the preserved simulation policy.
- `PathPlanner.select_goal()` currently selects only stored graph states whose
  end-effector lies between target_clearance=0.04 m and goal_tolerance=0.12 m
  from the requested target. States closer than 0.04 m are deliberately skipped.
  There is no Cartesian IK refinement in this goal-selection function.
- A controlled graph-node target probe requests graph node 3157's exact,
  valid end-effector position. The selector skips that zero-error state and
  chooses node 2780, at 0.046149 m from the requested position. This confirms
  the existing target stand-off semantics independently of the live fault.
- Motion COMPLETED checks final JOINT error against the selected joint goal
  (currently 0.02 rad tolerance). It does not establish that the end-effector
  has reached the requested Cartesian target center.
- The red target is also a collidable box in MuJoCo, with half-sizes
  0.035/0.020/0.025 m and contact masks enabled. Keep target contact checking
  active; changing center/approach semantics requires an explicit tool/contact
  goal definition, not just disabling the target collider.
- The green GNG workspace samples do not certify every interior location,
  joint-space connection, whole-arm clearance, or successful execution.

Diagnostic correction:

- `PlanningNode.report_runtime_diagnostics()` logs a Planning snapshot every
  five seconds, even when status does not change. It reports Planning/Motion
  status, failure reason, fault/STOP ownership/pending command, planning flag,
  path count, feedback/obstacle readiness, human checking state, latest search
  failure, requested target/current end-effector and actual/selected-goal target
  errors in meters. Existing state transitions and command decisions are intact.
- Distance queries use the visualization scratch model, leaving it at measured
  joints; they do not mutate CollisionFilter or PlanningManager. Diagnostic FK
  errors are logged without interrupting command dispatch. Checks with actual
  MuJoCo and stub transport verified fault visibility, distance reporting,
  five-second throttling, clock reset, missing inputs and nonfatal FK failure.
- Syntax, Flake8 (99-character limit) and Git whitespace checks passed. No
  ROS launch or live movement was executed here; no new end-to-end claim.

To obtain the stored status from a running stack, use transient-local durability
because these status publishers only publish on changes. The Motion topic is
`/motion/execution_status`, not `/motion/status`:

```bash
ros2 topic echo /planning/status --once --qos-durability transient_local
ros2 topic echo /motion/execution_status --once --qos-durability transient_local
ros2 topic echo /motion/failure_reason --once --qos-durability transient_local
```

Remaining live hypotheses include a latched control fault/failed recovery,
unowned STOP, unchanged failed graph request, or an already completed selected
stand-off goal. The current excerpt cannot choose between them. Do not clear
faults, release an external STOP, or relax collision margins based on this image.

### 2026-11-14 — One configured separation margin and faster retreat screening

Status: Implemented and tested headlessly after the user's request to keep the
extra separation margin in Planning rather than also padding human geometry.
Live ROS/Kinect execution remains unverified here.

The supplied follow-up log confirms SPHERE_AND_MESH_BOX is active. At constant
robot joints, refined human clearance ranges from about 0.017 to 0.063 m, with
one FAR report at 0.172 m. Retreat/search failures repeatedly coincide with
HUMAN_DATA_UNKNOWN, STALE_MESSAGE and data expiry during graph search. The old
HUMAN_DATA_OR_STATE_CHANGED reason is also consistent with expiry between two
consecutive candidate checks; these callbacks do not mutate geometry during
the synchronous search. The log does not establish that margin alone prevents
escape or that a certified path exists through the current human geometry.

Changes:

- `cra/config/reconstruction.yaml`: set `capsule.safety_margin` from 0.015 to
  0.0 for the CRA experiment. Anatomical minimum/maximum radii, voxel-derived
  estimates, smoothing and ROI settings remain unchanged. Human bodies are
  still represented by positive-radius capsules. Planning's human separation
  margin remains 0.10 m; static/contact margin remains 0.005 m. Removing the
  Vision padding can reduce total conservatism by up to 0.015 m, subject to
  radius clamping/filtering. It does not establish physical deployment safety.
- This is a CRA YAML override, not a change to Vision's C++ standalone defaults.
  `vision/reconstruction.launch.py` already loads that CRA configuration under
  `reconstruction_node`. Rebuild/source `cra` and restart reconstruction to
  load it and reset previously filtered radii. Verify with
  `ros2 param get /reconstruction_node capsule.safety_margin` (expected 0.0).
- `PathPlanner.find_human_margin_retreat()` now screens candidate endpoints
  before sampling their complete edges. Endpoints with no improvement or with
  worse clearance for another protected pair cannot win or pass the existing
  certificate. Skip these unnecessary edge checks, then run the complete
  sampled certificate for every potentially winning candidate. The final
  straight segment and all execution checks remain required.
- An invalid/static-contact endpoint rejects only that candidate. Expired data
  or a changed revision aborts with the explicit data/environment reason.
  No timeout, human-data freshness rule, search-step budget, GNG/A* algorithm,
  collision margin or execution experiment flag was relaxed.

Verification:

- All 74 tests passed: 70 Planning/geometry/visualization tests plus four
  simulation hold tests. Three new regressions verify rejection of an invalid
  candidate without losing other directions, inability to execute an improving
  endpoint without the full edge certificate, and fail-closed expiry during
  candidate screening. `/tmp/cra_single_margin_and_retreat_tests.log` records
  the Planning suite.
- A fixed eight-capsule comparison returns the identical retreat before/after
  screening. Full edge checks fall from 271 to 44, state checks from 923 to 497,
  and measured time from approximately 0.469 to 0.247 s. A separate real-clock
  probe with 0.35 s of capture lifetime remaining rejects the old search after
  expiry and returns a still-fresh path in 0.219 s with screening. These local
  controlled probes are not timing guarantees for the supplied live scene.
  Results: `/tmp/cra_retreat_screening_benchmark.log` and
  `/tmp/cra_retreat_remaining_deadline_probe.log`.
- Config/launch checks confirm positive body radii, CRA parameter-file loading,
  zero extra Vision padding and the unchanged 0.10 m Planning margin. A finite
  capsule/box example changes from 0.095 m (NEAR) to 0.110 m (FAR) when only
  the extra 0.015 m radius padding is removed. Logs are in
  `/tmp/cra_single_margin_config_check.log`.
- Flake8 (99-character limit), syntax and Git whitespace checks passed. Full
  ROS build/transport and live movement were not run in this environment.

The narrower padding alone cannot clear the supplied 0.05286 m state under a
0.10 m margin (adding 0.015 m yields only 0.06786 m). Retreat still needs fresh
human geometry and a certified non-worsening path. Long general A* searches,
upstream invalid/stale detections, capsule motion/noise and joint tracking
failures remain separate limitations; this change does not guarantee escape.

### 2026-11-14 — Mesh-enclosing boxes for human collision refinement

Status: Implemented and tested headlessly after the user authorized the geometry
correction. Native ROS/RViz and live human avoidance remain unverified here.

Changes:

- `human_collision.mesh_local_box()` derives an enclosing box from every compiled
  MuJoCo mesh vertex, using the geom-local frame after MuJoCo mesh scaling,
  centering and rotation. Boxes have 1 micrometer of numerical enclosure padding.
  The configured 0.10 m human margin is unchanged.
- `CollisionFilter` caches boxes for the same five active arm mesh geoms and
  transforms them with each geom's measured/planned pose. Unsupported non-mesh
  geoms retain sphere checking. State, edge, path and human-margin retreat checks
  all use the shared human-distance implementation.
- Sphere checking remains the broad phase. Potential proximity is refined with
  finite human-segment/oriented-box distance, minus the human capsule radius.
  The larger sphere/box clearance lower bound is still conservative because
  BOTH volumes enclose the mesh. Thus a real mesh contact cannot be made clear
  merely by discarding the old sphere's empty space.
- `segment_box_distances()` minimizes squared distance on all piecewise
  quadratic intervals of the finite segment, split at box-face crossings.
  It covers point capsules, parallel segments, tangencies, corners and segments
  whose endpoints lie outside but whose middle crosses the robot enclosure.
- Sphere-clear checks skip unnecessary box-distance computation. Retreat's
  all-pair reports always use refined distances, including FAR pairs, to avoid
  discontinuities in the non-worsening clearance certificate. Invalid potential
  collision boxes/non-finite distance results return UNKNOWN.
- Diagnostics add geometry_model, sphere_clearance and box_clearance. The
  existing robot_radius remains the broad-phase sphere radius; subtracting that
  radius from center_distance alone no longer describes the refined clearance.
- RViz's optional `/planning/robot_collision_bounds` display now includes five
  green mesh boxes plus the existing five blue spheres, using the same bounds
  helper as CollisionFilter. The measured grey robot mesh is unchanged. The
  config display is named "Robot collision bounds - spheres and mesh boxes".
- Regression fixtures now place close/overlapping human capsules at the actual
  requested proxy clearance, rather than assuming a sphere radius still defines
  the refined surface. Their original safety assertions are preserved.

Verification:

- All 71 tests passed: 53 existing collision/planning regressions, 11 new
  distance/enclosure tests, three RViz marker tests and four simulation hold
  tests. Results: `/tmp/cra_mesh_box_all_tests.log` and
  `/tmp/cra_mesh_box_hold_tests.log`.
- Every vertex in all five meshes is enclosed at zero and eight randomized
  joint configurations. Another 100 samples put capsule centers on actual mesh
  vertices and confirm they remain blocked. Box distance matches an independent
  scalar convex minimization for 300 random segments.
- The prior deterministic synthetic witness changes from sphere NEAR at
  0.0700 m to refined FAR above 0.22 m, with the same 0.10 m human margin.
  Separate tests verify the unchanged threshold on both sides of 0.10 m.
- Actual GNG/A* detour search and STOP/RELEASE/EXECUTE regressions pass with
  OBSERVED geometry. The frozen-clock detour is 6.294 rad versus the original
  4.543 rad, with sampled refined minimum clearance 0.1004 m. Its search takes
  about 14.2 s here: this is an offline geometric result, NOT live freshness or
  end-to-end avoidance validation within the existing 0.5 s deadline.
- One-capsule 1,000-query measurements took about 0.087 s for the prior sphere
  implementation versus 0.233 s with refinement. More precise checking adds
  work; the sphere-clear fast path avoids needless narrow checks. Measurements
  are local, not timing guarantees. `/tmp/cra_mesh_box_benchmark.log` records them.
- Python syntax, Flake8 (99-character limit), config parsing and CRLF-aware Git
  whitespace checks passed. No safety margin, timeout, GNG/A* algorithm, ROS
  message or experiment flag was changed. Legacy workspace_planning is untouched.

Rebuild `cra planning`, source the overlay, and restart CRA. Verify the unchanged
0.10 m margin, green boxes enclosing the grey robot, sphere-vs-box diagnostics,
and collision behavior with continuously OBSERVED human geometry. Remaining
VISION_INVALID/expired snapshots, historical-path validation and joint-2
TRACKING_ERROR findings from the audit below are separate issues; this change
does not guarantee motion in every human configuration or certify physical use.

### 2026-11-14 — Collision sensitivity audit after RViz confirmation

Status: Diagnostic audit and a headless geometry counterexample completed.
The user confirms RViz robot rendering now works. No additional production
source, collision geometry, safety margin, or timeout changes in this audit.

The latest attached run has 17 HUMAN_PATH_BLOCKED, 23 HUMAN_RETREAT_BLOCKED,
18 HUMAN_RETREAT_DATA_UNKNOWN, and 19 HUMAN_VISION_UNKNOWN status transitions.
Failed-retreat diagnostics include 21 HUMAN_DATA_UNKNOWN and 12 HUMAN_OVERLAP
reports. Counts describe logged transitions/reports, not durations or all cycles.
There are also 34 graph-search data-expiry messages, 114 rejected geometry
messages, and two Motion TRACKING_ERROR failures for joint 2 (0.318 and
0.423 rad errors with velocity filtered to 0.500 rad/s). Not every STOP is
caused by human collision proximity.

Confirmed geometry conservatism:

- `CollisionFilter._human_collision_after_forward()` represents each arm mesh
  as one sphere using `geom_rbound`. `nearest_human_collision()` checks the
  sphere against human capsules and applies the configured 0.10 m human margin.
- The link2 sphere radius is approximately 0.19917 m (diameter 0.39834 m),
  whereas its compiled mesh-local box spans approximately
  0.09531 x 0.12647 x 0.32842 m. A sphere around a long, narrow link includes
  substantial empty space beside the link.
- A deterministic synthetic point-capsule witness at a joint pose taken from
  the latest log gives sphere clearance 0.0700 m (NEAR), while its surface
  clearance to the enclosing oriented boxes of ALL FIVE checked arm meshes
  is at least 0.22139 m. Since each box encloses its mesh vertices and triangles,
  this is a conservative lower bound on mesh clearance, above the same 0.10 m
  margin. This proves a possible false proximity classification in the current
  proxy; it does not reconstruct the live human's unavailable capsule endpoints
  or establish that all logged overlap classifications are false.
- Reproduction output and witness coordinates are in
  `/tmp/cra_collision_sensitivity_probe.json` and
  `/tmp/cra_collision_sensitivity_probe.log`. They use the real MuJoCo model,
  existing frame conversion and unchanged human checking implementation.

Other confirmed mechanisms and remaining hypotheses:

- HUMAN_PATH_BLOCKED can occur with a FAR current-pose report. Current-pose
  separation does not establish clearance along a future trajectory. The
  source checks state, full path, revision and freshness; failure of any is
  grouped under HUMAN_PATH_BLOCKED, so the log does not identify the failing
  waypoint/edge or distinguish expiry during that check.
- During active execution `_human_guard()` revalidates the full
  `last_execution_path`, including already traversed portions. A human near a
  past portion may cause unnecessary stopping; the latest logs do not include
  progress/capsule endpoints sufficient to prove that case occurred live.
- Planning searches execute synchronously in `planning_cycle()` with ordinary
  `rclpy.spin(node)`. Long searches can delay feedback/Vision callbacks while
  the existing 0.5 s capture deadline expires. This is an additional freshness
  mechanism, distinct from upstream VISION_INVALID messages. Increasing the
  timeout alone would conceal the age rather than establish fresh geometry.

Minimal proposed corrections, not implemented by this audit:

1. Retain the sphere as a broad-phase bound; refine potential proximity with
   a proven enclosing oriented box or capsule for each arm link. Prove mesh
   containment and finite-segment distance correctness before enabling it.
   Preserve the 0.10 m human margin and existing static/contact checks.
2. Preserve timely receipt of human/feedback snapshots during long searches,
   with explicit ownership of MuJoCo/planner state. Do not simply make shared
   collision state concurrently mutable or accept expired geometry.
3. Report the actual failing path edge/state and expiry reason. Validate the
   remaining path using trustworthy execution progress, without guessing past
   self-intersections or skipping unexecuted segments.
4. Investigate the joint-2 tracking failures separately; do not raise its error
   limit merely to avoid STOP. Confirm avoidance using continuously OBSERVED
   data rather than UNKNOWN-ignored simulation execution.

### 2026-11-14 — Staged human retreat and gravity-balanced simulation STOP

Status: Implemented and tested headlessly. Live ROS/Motion/Kinect behavior and
native RViz rendering remain unverified here.

The latest supplied run sends four HUMAN_RETREAT_SENT trajectories, so retreat
does reach Motion. Subsequent cancellations include STALE_MESSAGE, changed
capsule geometry, and "Measured robot motion reduced human clearance". There
are also 27 HUMAN_OVERLAP reports and three SEARCH_BUDGET_EXHAUSTED reports.
These are distinct from the previously fixed EMPTY recovery latch.

Changes:

- `RobotSimulation.hold_position_target()` balances the configured CRA unit-gear
  position actuators using model bias forces and actuator gains when capturing
  a hold target. Startup, zero command, invalid command, and command timeout use
  the same bounded target. Repeated zero commands still do not recapture it.
  Previously setting the position-actuator target to the measured angle removed
  its supporting torque, allowing gravity to move an otherwise stationary arm.
  No contact force compensation, model/gain changes, or joint-limit changes
  were introduced.
- A controlled 0.5 s probe at the supplied pose
  `[1.98576289, -0.64575674, 1.22786207, -0.91740616, -0.94390645]` measured
  0.07318 rad maximum drift with the old hold, versus below 1e-9 rad with the
  gravity-balanced target. This starts at rest and does not prove instantaneous
  braking of a moving arm.
- `PlanningNode.joint_callback()` now passes named measured joint velocities to
  `PlanningManager`. Malformed/non-finite supplied velocities cannot refresh
  feedback. Position-only callers retain compatibility, with velocity estimated
  when two timed positions are available.
- Retreat search and the pre-execution guard reject measured speeds above
  `recovery_stability_tolerance / recovery_settle_time` (currently 0.02 rad/s),
  reporting ROBOT_STILL_MOVING or WAITING_FOR_ROBOT_SETTLE. A STOPPED
  acknowledgment alone establishes trajectory cancellation, not physical rest.
  This is a measured-speed gate, not a full braking-distance certificate.
- `PathPlanner.find_human_margin_retreat()` still prefers an endpoint outside
  the human margin. After the existing bounded search, it may return a straight
  partial retreat with at least the existing static safety margin's worth of
  clearance improvement (currently 0.005 m). Its endpoint can remain NEAR.
- Partial endpoints use the existing special human-margin certificate: valid
  joints, no static/MuJoCo contact, positive clearance for every human pair,
  fresh geometry, and sampled non-worsening clearance for every initially-close
  pair, without entering another pair's margin. The full straight segment must
  pass; monotonic search steps alone are insufficient. Manager revalidates the
  certificate before RELEASE/EXECUTE and during execution. On partial completion
  it requests STOP and plans the next retreat from measured state; the original
  Cartesian target is retained. Ordinary GNG/A*/state/edge/path checking and
  configured margins are unchanged. Overlap or unknown data still blocks retreat.
- `PlanningVisualizer` logs its six mesh markers' topic/frame at startup and
  confirms the first measured publication. Existing robot-marker numerical
  tests pass; no renderer failure was established from the supplied log.

Verification:

- 56 Planning/visualization tests and four simulation hold tests passed. New
  checks exercise staged completion/retry, expired partial certificates,
  measured braking before search and during RELEASE, invalid velocity feedback,
  and the actual simulation zero/timeout/invalid-command hold callbacks.
- A simplified dynamics probe with one stationary capsule and a bounded partial
  segment increased measured clearance from 0.0200 to 0.02774 m in 0.2 s with
  no contacts or sampled non-worsening violations. It uses a linear reference
  and the actual simulation callbacks, not the complete C++/ROS trajectory stack.
- Results: `/tmp/cra_staged_retreat_tests.log`,
  `/tmp/cra_staged_motion_probe.log`, and `/tmp/cra_hold_control_tests.log`.
- Flake8 (99-character limit), Python syntax, and CRLF-aware Git whitespace
  checks passed. ROS/colcon and GUI execution are unavailable here.

RViz investigation: the user's saved `~/.rviz2/default.rviz` lacks a robot-model
display, while the installed `cra/config/cra.rviz` symlink points to the correct
source configuration. The user reports launching with `rviz:=true`, so the old
default config alone is not an established cause. All six binary STL resources
exist and contain finite vertices and unit normals. Actual display status,
resource loading and viewport visibility remain to be checked on the ROS host.
Open the exact source config to remove overlay/config selection uncertainty:

```bash
rviz2 -d ~/living-lab/cra_ws/src/cra/config/cra.rviz
```

Rebuild `cra planning simulation`, source the overlay, and restart the stack.
Verify first-publication logging and the robot MarkerArray display; then test
retreat with a stationary, positively separated capsule before moving humans.
Compare the robot mesh and diagnostic collision spheres against human capsules
if negative model clearances disagree with physical separation. The remaining
VISION_INVALID/STALE_MESSAGE stream and human motion can still interrupt retreat;
these changes do not establish guaranteed escape or end-to-end human avoidance.

### 2026-11-14 — Release simulation recovery after fresh valid EMPTY

Status: Implemented and tested headlessly. Live ROS/Motion/Kinect execution
remains unverified in this environment.

The latest supplied run reports workspace SAFE at 1794634696.462, followed by
Human Vision EMPTY (NO_ACTIVE_CAPSULES) at 1794634696.484, yet Planning stays in
HUMAN_RETREAT_DATA_UNKNOWN. The recovery latch required OBSERVED geometry even
after a fresh validated EMPTY. CollisionFilter deliberately maps EMPTY to
UNKNOWN, so the latch returned before the ordinary simulation policy could
resume planning. Three reproducing tests failed before the correction.

Changes:

- `PlanningManager` retains a separate EMPTY deadline, bounded by both the
  snapshot capture expiry and the existing human-data receipt timeout.
- With `simulation_only_ignore_unknown=True`, fresh validated EMPTY ends the
  close-human recovery latch and discards the retreat path. Any active retreat
  or pending RELEASE first requests STOP and waits for acknowledgment and fresh
  joint feedback. Normal planning then targets the original requested position.
- This does not itself issue RELEASE or EXECUTE. Ordinary joint, static-contact,
  edge and path validation and the owned STOP/RELEASE/IDLE handshake still apply.
  External/unowned STOP cannot be automatically released.
- UNKNOWN/VISION_INVALID, expired EMPTY, and missing data cannot clear a known
  close-human stop. Strict mode does not use this simulation-only EMPTY rule.
  GNG, A*, safety margins and experiment flag defaults are unchanged. An EMPTY
  simulation execution is not evidence of successful human obstacle avoidance.

Verification:

- All 51 headless tests passed: 48 collision/planning tests and three robot
  visualization tests, using the real MuJoCo model and graph with explicit
  Motion acknowledgments. Six new regressions cover EMPTY after a blocked or
  active retreat, capture/receipt expiry, subsequent invalid data, static
  contacts, and external STOP ownership.
- Flake8 (99-character limit) and CRLF-aware Git whitespace checks passed.
- Test output: `/tmp/cra_empty_resume_tests.log`. ROS/colcon build and live
  movement were not executed here. Rebuild `planning`, restart the stack, and
  verify a fresh EMPTY after human removal permits PATH_READY followed by the
  normal RELEASE/IDLE/EXECUTE sequence when the original target needs motion.

### 2026-11-14 — RViz robot mesh and collision throughput after live retreat timeout

Status: Implemented and tested headlessly. ROS launch, RViz rendering, and
end-to-end execution with the live Kinect are still unverified here.

The latest supplied run contains one HUMAN_RETREAT_READY, immediately followed
by MESSAGE_TIMEOUT before any retreat trajectory is sent. Positive-clearance
attempts repeatedly consume about the entire 0.5 s freshness window. It also
contains negative model clearances (for example -0.0926 m at link3), which the
non-contact retreat deliberately rejects. Faster computation addresses the
timeout mechanism; it does not establish that an overlapping model is safe.

Changes:

- `human_collision.py`: batch all robot-sphere/human-capsule distances in NumPy
  using the same frame conversion, finite-segment projection, radii and margin.
  Preserve nearest-pair ordering and optional all-pair clearance diagnostics.
  Differential tests compare every pair against the prior scalar calculation,
  including degenerate capsules and malformed robot geometry.
- A controlled eight-capsule comparison using the same start and geometry found
  the identical retreat in 0.180 s with the scalar calculation and 0.079 s with
  batching. Both retained the existing 0.5 s deadline. An isolated 2,000-query
  comparison took about 0.585 s versus 0.074 s. These are local measurements, not
  guarantees for live or more constrained scenes; difficult graph searches can
  still exceed the deadline.
- `HumanSnapshot` now optionally carries an absolute monotonic expiry derived
  from both receipt age and ROS capture age. CollisionFilter honors that expiry.
  Copying a still-OBSERVED, nearly expired message no longer grants it another
  full 0.5 s. For example, data already 0.4 s old retains about 0.1 s of validity.
  Synthetic snapshots without the optional field retain their existing behavior.
  No ROS message schema, timeout setting, or experiment flag was changed.
- `PathPlanner` and `PlanningManager` retain the reason for a failed retreat.
  Changed-condition logs now distinguish retreat_reason values such as
  HUMAN_OVERLAP, HUMAN_DATA_UNKNOWN, NO_MONOTONIC_STEP, and
  SEARCH_BUDGET_EXHAUSTED, separately from the human collision report's reason.
- `PlanningVisualizer` publishes the measured arm as mesh markers on
  `/planning/robot_model` and diagnostic bounding spheres on
  `/planning/robot_collision_bounds`, with transient-local depth-one QoS.
  Updates use named `/joint_states` feedback at at most 10 Hz on the existing
  callback; no extra node/timer/thread or robot_description/TF publisher is used.
- All six robot STL meshes, including the base, are displayed in CRA base_link.
  Marker poses undo MuJoCo's mesh centering/principal-axis rotation before
  rendering the original files, and retain each XML mesh scale. The optional
  spheres match the same five active arm geoms and geom_rbound values used for
  human checking. Mesh resources use local file URIs, so RViz needs access to
  the same installed/source mesh files when run on a different machine.
- Added `src/cra/config/cra.rviz`: robot mesh, human capsules, target, and
  simulation obstacles enabled; collision spheres and GNG overlays available
  but initially hidden to keep the arm visible. It uses Fixed Frame=base_link.
- Main launch accepts `rviz:=true` to open that configuration. Default is false,
  preserving existing launches and avoiding an extra window when RViz is
  already open. `cra/package.xml` declares the RViz2 runtime dependency.

Verification:

- 45 headless tests passed: 42 collision/planning regressions and three robot
  visualization tests with real MuJoCo and mocked ROS transport.
- Numerical tests verify mesh position/orientation against the original XML at
  zero, an observed pose, and a randomized joint configuration to 1e-12 tolerance;
  mesh scales/resources, five collision spheres, named-joint reordering,
  feedback throttling, clock reset, and malformed feedback are also checked.
- A new regression verifies copying a snapshot cannot restart its capture
  timeout. Existing all-pair retreat/STOP/RELEASE tests remain passing.
- A mocked-launch check verified rviz true/false, the installed config argument,
  topic/QoS settings, package dependency, and preservation of existing CRLF files.
  Python AST, Flake8 (99-character limit), XML/YAML parsing, and CRLF-aware
  `git -c core.whitespace=cr-at-eol diff --check` passed for changed files.
- Temporary results: `/tmp/cra_robot_and_collision_tests.log` and
  `/tmp/cra_collision_speed_comparison.log`. Native RViz mesh loading, DDS QoS
  matching, ROS package build, and live movement were not executed here.

Build both the updated config/launch package and Planning on the ROS workstation:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cra planning --symlink-install
source install/setup.bash
ros2 launch cra cra.launch.py rviz:=true
```

For an already running stack, load the installed `cra/config/cra.rviz` using
RViz File -> Open Config, or add a MarkerArray display for
`/planning/robot_model` with Reliable/Transient Local QoS and Fixed Frame
`base_link`. Enable "Robot collision spheres - diagnostic" to compare the
conservative spheres with the actual arm mesh and the orange human capsules.
This is a measured mesh display, independent of `/robot_description`
and the Effort display; it does not publish robot motion commands.

Reproduce the combined offline tests from the workspace root:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/planning:src/simulation \
  python3 -m unittest discover -s src/planning/test -p 'test_[dr]*.py' -v
```

In this diagnostic environment prepend `/tmp/cra-debug-deps:` to PYTHONPATH.
Keep configured margins and both experiment flags unchanged. If the arm mesh
and human geometry disagree physically, verify camera calibration and capture
the capsule endpoints before proposing tighter conservative collision geometry.
Negative-clearance recovery remains blocked, and improved throughput alone does
not resolve upstream VISION_INVALID messages or prove dynamic human avoidance.

### 2026-11-14 — Retreat from a human safety buffer in controlled simulation

Status: Implemented and tested with headless geometry/state-machine tests and
one simplified MuJoCo dynamics probe. Live ROS/C++ execution remains unverified.
This extends the earlier stop/replan work after the request to seek a safe
position before returning to the original target.

The new behavior distinguishes an existing safety-buffer violation from model
penetration. A small retreat is considered only when EVERY robot/human pair has
positive clearance and MuJoCo reports no robot contact or static-margin contact.
A negative/zero human clearance, invalid joints, unknown human geometry, or a
failed retreat certificate keeps the robot stopped. In particular, the negative
clearances in the user's latest run are not treated as permission to move.

Implemented behavior:

1. On HUMAN_TOO_CLOSE, request STOP, confirm STOPPED, and receive fresh joint
   feedback. An external/unowned STOP is never automatically released.
2. Search up to 32 local steps of at most 0.01 rad, with 10 coordinate directions
   per step. Every candidate respects joint limits, max_joint_delta, and Motion's
   max_segment_delta. The main GNG/A* search is unchanged.
3. Check every robot-geometry/human-capsule pair at interpolated samples no more
   than 0.005 rad apart. Initially close pairs must not lose clearance; initially
   clear pairs must stay outside the original human margin. Once a pair exits
   the buffer it may not re-enter. Only 1e-9 m numerical comparison tolerance is
   used; no configured radius or margin was reduced.
4. Accept only a straight two-waypoint segment whose endpoint is normally valid
   and clears the human margin plus the existing 0.005 m static margin. Ordinary
   `is_state_valid()`, `is_edge_valid()`, and `is_path_valid()` still reject an
   initial state inside the human margin. Recovery uses a separate, explicit
   non-contact/monotonic-clearance certificate rather than skipping validation.
5. Validate before RELEASE, confirm IDLE, and validate again before executing.
   Re-evaluate the remaining straight segment using the newest observed human
   geometry and measured joints. Safe geometry revisions can continue the move;
   worsening measured progress, contact, an unsafe endpoint, or missing/stale
   feedback/geometry requests STOP.
6. When measured clearance is FAR, stop at the safe configuration, confirm fresh
   feedback, and return to ordinary GNG/A* planning for the unchanged target.
   Retreat completion also invalidates the staging path and requests normal
   replanning. No permanent home/safe pose is assumed to be clear.

Integration and scope:

- `CollisionFilter.human_margin_state()` exposes all pair clearances and rejects
  overlap/contact. `is_human_margin_transition_valid()` implements the separate
  recovery certificate. `human_collision.py` optionally includes all pairs in
  its existing calculation; the default nearest-pair calculation is retained.
- `PathPlanner.find_human_margin_retreat()` provides the bounded local search.
- `PlanningManager` adds HUMAN_RETREAT_READY, HUMAN_RETREAT_SENT,
  HUMAN_RETREAT_CLEAR, HUMAN_RETREAT_BLOCKED, HUMAN_RETREAT_UNSAFE, and
  HUMAN_RETREAT_DATA_UNKNOWN statuses, retaining the existing Motion protocol.
- The manager's new `human_margin_recovery_enabled` argument defaults to False.
  PlanningNode enables it only for `robot.mode: simulation`, and the manager
  additionally requires the existing simulation experiment flag to be True.
  It is disabled for physical robot mode. No new node, thread, or timer was added.
- Initially this recovery context required fresh OBSERVED geometry after both
  UNKNOWN and EMPTY. The later "Release simulation recovery after fresh valid
  EMPTY" correction above supersedes the EMPTY restriction: a validated, fresh
  EMPTY may return to ordinary simulation planning. UNKNOWN still cannot
  authorize a blind retreat. `simulation_only_ignore_unknown=True` and
  `execute_path=True` remain unchanged.
- Failed searches are not repeated for identical geometry, readiness, and joint
  state. New attempts also wait at least the existing 0.5 s recovery settle time.
  Fresh identical geometry may retry after a data-expiry failure.

Verification:

- The suite now has 39 passing tests (the earlier 24 plus 15 retreat regressions).
  New cases cover leaving a positive-clearance buffer, preserving all pair
  distances, rejecting a new margin violation, preserving normal collision
  rejection, returning through STOP/RELEASE to real A* and the original target,
  safe/unsafe new human snapshots, measured motion in the wrong direction,
  unknown data, expiry/retry, external STOP, joint limits, MuJoCo contacts,
  overlap, and bounded failed searches.
- A controlled model case starts at 0.075 m clearance and finds an endpoint at
  approximately 0.1068 m without changing the 0.10 m human margin. The local path
  changes joint1 by about -0.11 rad. Dense checks cover all pairs along the move.
- Real-clock local-search probes found this retreat in about 0.057 s with one
  capsule and 0.172 s with eight capsules (seven deliberately far away), keeping
  the existing 0.5 s freshness deadline. These isolated timings are not a
  real-time guarantee for the live Vision/ROS pipeline or constrained geometry.
- A separate 0.4 s headless MuJoCo dynamics probe used a stationary capsule,
  gravity-balanced initial hold, linear velocity reference, and a simplified
  implementation of the configured tracking/position-target equations. Model
  clearance increased from 0.075 m to 0.1086 m, with no sampled non-worsening
  violations and no MuJoCo contacts. This was not execution of the C++ Motion
  component. Probe artifacts are `/tmp/cra_margin_physics_probe.py` and `.log`;
  its temporary fixture is `/tmp/cra_human_margin_fixture.json`.
- Changed Python files pass AST/Flake8 checks and `git diff --check`. A broader
  package lint invocation also reported pre-existing missing final newlines and
  trailing whitespace in untouched files; those files were not changed.
- Full test output is `/tmp/cra_human_retreat_tests.log`. Reproduce using the
  unittest command in the following entry; tests explicitly enable margin
  recovery only for their controlled cases. No ROS commands are published.

Build/restart Planning on the ROS workstation to try the new simulation flow:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select planning --symlink-install
source install/setup.bash
ros2 launch cra cra.launch.py
```

Verify the startup log contains `human_margin_recovery=True` in simulation.
Record the normal joint, human geometry, planning/Motion status, STOP, trajectory,
and velocity topics. Test a stationary human outside contact but inside the
buffer first, then changed geometry and data dropout. Live sensor calibration,
tracking/braking between checks, arbitrary moving-human retreat, and end-to-end
completion are not validated by the headless tests. The method can reject a
feasible retreat because of its local search and conservative pair constraints;
it does not guarantee a safe escape exists or that the robot can always move.

No changes were made to Motion, Vision, Simulation, GNG/A*, legacy packages,
robot/model configuration, joint limits, or configured safety margins.

### 2026-11-14 — Dynamic human obstacle replanning and safe resume

Status: Implemented and partially tested with the real MuJoCo model and GNG
joint graph in headless Python tests. Live ROS 2 transport, the C++ Motion
controller, and end-to-end dynamic human avoidance remain unverified. This
feature is not marked complete for live experiments.

Verified diagnosis before implementation:

- `PlanningManager.step()` set `planning_required=False` before goal selection,
  but did not remember a failed goal as blocked. A human could block every goal
  sample, move away, and leave Planning indefinitely at `GOAL_UNREACHABLE` even
  though a valid goal and path existed. Reproduced with the real planner/model.
- With `simulation_only_ignore_unknown=True`, `_human_guard()` retried an
  unsuccessful plan on every tick while EMPTY/UNKNOWN remained unchanged.
  A regression test observed four identical searches instead of one.
- The STOPPED branch sent RELEASE before `prepare_execution_path()` and final
  collision validation. Injecting a preparation failure demonstrated that STOP
  was released before a safe executable trajectory had been established.
- Feedback received before the STOPPED acknowledgement could be reused for
  the next plan. STOPPED now requires a new accepted joint measurement.
- `human_collision_report()` returned UNKNOWN for invalid joint input without
  identifying that cause, so fresh human data could be labelled unknown Vision.
- A fixed human capsule blocks a baseline path while the original A* finds a
  longer, valid alternative using existing graph edges. The controlled fixture
  has joint-space lengths approximately 4.543 rad before obstruction and
  6.352 rad for the alternative. Expanded graph connections and clearance costs
  were not needed for this reproduction and were not added.
- That alternative search takes roughly nine seconds on a fixed snapshot in
  this environment. With the real monotonic clock and the unchanged 0.5 s
  human timeout, an offline attempt expired after about 0.52 s and was formerly
  misreported as `NO_GRAPH_PATH`. Expired geometry must not be diagnosed as
  evidence of insufficient graph connectivity.

Saved runtime evidence inspected:

- `~/.ros/log/python3_605_1794607182571.log` records five TRAJECTORY_SENT and
  EXECUTING events, one COMPLETED event, and subsequent HUMAN_TOO_CLOSE,
  HUMAN_PATH_BLOCKED, GOAL_UNREACHABLE, and RECOVERY_BLOCKED states.
- `~/.ros/log/component_container_603_1794607182214.log` confirms trajectory
  acceptance and cancellation after STOP, with successful RELEASE events.
- `~/.ros/log/python3_599_1794607182574.log` records nonzero commands, actual
  joint movement, later zero commands, and contacts after target relocation.
  These are historical observations, not a new end-to-end test of this change.

Files and behavior changed:

- `src/planning/planning/planning_manager.py`: remember failed goals/searches
  with their environment revision, human readiness, and measured joint state.
  Retry after geometry/readiness changes or measured movement beyond the
  existing 0.01 rad recovery stability tolerance. Repeated identical failures
  under the simulation unknown policy wait for a changed condition. Every
  actual geometry update still invalidates collision caches as before.
- The same manager now requires post-STOPPED joint feedback; validates and
  prepares a trajectory before RELEASE; waits for IDLE acknowledgement; and
  revalidates before EXECUTE. It checks feedback and obstacle freshness again
  after potentially expensive validation. A blocked or invalid path does not
  release STOP. A new obstruction during a pending RELEASE requests STOP.
- NEAR still means an invalid current configuration under the existing human
  margin. The redundant branch attempting ordinary replanning from NEAR was
  removed. Human contact retreat remains disabled; no recovery was enabled.
- `src/planning/planning/path_planner.py`: expired human data during graph,
  fallback, or final validation is reported as HUMAN_VISION_UNKNOWN. Do not run
  fallback searches against expired data. Invalidate A*'s revision cache after
  expiry so fresh, identical geometry cannot reuse timeout-induced negative
  entries. Start/goal rejection logs include joints, environment revision, and
  the existing `diagnose_state()` report.
- `src/planning/planning/algorithms/collision_filter.py`: add reason fields to
  UNKNOWN human reports to distinguish HUMAN_DATA_UNKNOWN and INVALID_JOINTS.
  Collision geometry, margins, limits, and NEAR/FAR calculations are unchanged.
- `src/planning/planning/planning_node.py`: log changed collision conditions
  with source human status, environment revision, joints, minimum clearance,
  robot geometry/radius, human IDs/radius, and margin. Repeated reports for the
  same blocking pair are suppressed; nearest-pair changes while FAR do not
  create extra logs. Previously swallowed manager exceptions now retain their
  stage, type, and message and are logged when the error changes.
- `src/planning/test/test_dynamic_replanning.py`: add 24 headless regression
  tests using the actual model, joint graph, and collision checker. Motion
  acknowledgements are supplied by the tests rather than a C++/ROS controller.

Geometry and data findings:

- The implemented chain is
  `p_CRA = C @ R_world_base.T @ (p_world_geom - p_world_base)`, with
  `C @ (x,y,z) = (x,-z,y)`. The rotation is proper and orthogonal. Forty sampled
  graph configurations agree with current-model FK to about 6.46e-8 m.
  Controlled capsules at transformed robot centers produce the expected NEAR
  result. This verifies internal numerical consistency, not physical Kinect
  extrinsic calibration.
- Human checking retains all five active arm collision meshes. `geom_rbound`
  spheres for link2/link3 are approximately 0.199/0.162 m. Their conservatism is
  visible in diagnostics, but no live false-positive pair was available to
  justify changing the representation or reducing any radius/margin.
- Monitor reasons in the logs include STALE_MESSAGE, MESSAGE_TIMEOUT, and
  VISION_INVALID. The monitor continues to distinguish OBSERVED/EMPTY/UNKNOWN.
  EMPTY maps to checker UNKNOWN; the preserved simulation flag may disable
  human checks for both EMPTY and UNKNOWN. Such executions do not establish
  successful observed-human avoidance.
- Capsule tuples are still compared exactly. Identical snapshots preserve the
  revision; any actual endpoint/radius change invalidates it. Numeric noise
  was not suppressed without a proven conservative bound: geometry changes
  must continue to revalidate safety. Small real changes can therefore still
  trigger repeated work and require profiling with recorded data.

Verification performed:

- The initial 15-test regression run failed with seven assertion failures and
  two missing-diagnostic errors. The corrected suite contains 24 passing tests.
- Covered no-human/far-human planning and completion state handling; blocked
  original path with a longer alternative; STOP/STOPPED/new-feedback/RELEASE/
  IDLE/EXECUTE; missing RELEASE acknowledgement; a new obstruction during
  RELEASE; no valid goal and subsequent obstacle removal; invalid starts;
  unchanged failed requests; status/timeout reasons; exception reporting; and
  data expiry during goal selection, search, and final validation.
- The detour is validated at states and interpolated edges with the unchanged
  collision checker. Sampled minimum sphere/capsule clearance exceeds the
  configured 0.10 m margin. This is model clearance, not physical separation.
- Python AST and Flake8 checks (99-character limit) passed for changed Python
  files. `git diff --check` passed.
- Tests used MuJoCo 4.0.1 and NumPy 2.5.4, with dependencies installed only under
  `/tmp/cra-debug-deps`. The actual ROS workstation's MuJoCo version was not
  established. Frozen monotonic time in deterministic snapshot tests isolates
  geometry and state transitions from computation latency; dedicated tests
  explicitly advance time to verify expiry. These tests do not prove real-time
  replanning under a live 0.5 s human-data deadline.

Reproduce the offline suite from the workspace root, using an environment
with NumPy, PyYAML, and MuJoCo installed:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/planning:src/simulation \
  python3 -m unittest discover -s src/planning/test \
  -p test_dynamic_replanning.py -v
```

In the current diagnostic environment prepend `/tmp/cra-debug-deps:` to
PYTHONPATH. The test file contains deterministic START, GOAL, and DETOUR
capsule coordinates; no ROS command is published by this suite. Initial and
corrected run outputs are `/tmp/cra_dynamic_before.log` and
`/tmp/cra_dynamic_after.log` (temporary investigation artifacts).

On the ROS workstation:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select planning --symlink-install
source install/setup.bash
ros2 launch cra cra.launch.py
```

Keep `execute_path=True` and `simulation_only_ignore_unknown=True`; both
PlanningNode defaults are unchanged. Observe these parameters with
`ros2 param get /planning_node execute_path` and
`ros2 param get /planning_node simulation_only_ignore_unknown`.

Record `/joint_states`, `/vision/human/geometry`, `/simulation/target_marker`,
`/simulation/obstacle_poses`, `/planning/status`, `/motion/joint_path`,
`/motion/stop_requested`, `/motion/execution_status`, `/motion/failure_reason`,
and `/motion/joint_velocity_command`. Repeat no-human, far-human, direct-route
blocking, obstruction during execution, fully blocked goal, and
OBSERVED/UNKNOWN/OBSERVED/EMPTY cases with equivalent starting states. Verify
the actual Motion acknowledgement and measured movement after each resumed
trajectory. Do not treat synthetic acknowledgements or PATH_READY as execution.

Remaining issues and next work:

- No ROS 2/colcon installation exists in this execution environment. The new
  logging and changed timing must be exercised with real DDS/ROS callbacks and
  the C++ Motion component. Live completion, stop counts, resume/completion
  success rates, and physical collision counts were not measured.
- Profile collision queries/search and callback scheduling before attempting
  real-time alternatives. The difficult fixture exceeds the live freshness
  deadline. No timeout was increased, no thread/timer/node was introduced, and
  no stale path was allowed to execute to hide this limitation.
- Human-path revalidation still conservatively includes the full commanded
  path. Motion does not publish an authoritative waypoint/progress index, so
  traversed prefixes were not discarded based solely on nearest joint pose.
  This can cause extra stops for obstacles behind the arm. Safe trimming needs
  reliable execution progress and tests for looping/nearby graph segments.
- Moving the collision-enabled target into the arm can independently create
  an invalid start or block every goal. Target placement validation remains a
  separate Simulation/GUI issue; those packages were not changed here.
- Validate physical camera calibration and inspect the new nearest-pair
  diagnostics before proposing tighter conservative whole-arm geometry.

No changes were made to Vision, Motion, Simulation, GNG, A*, workspace_planning,
robot/model configuration, joint limits, safety margins, or the two preserved
experimental settings.

### 2026-11-13 — Target GUI startup from Simulation launch

- Added `simulation.target_gui.enabled: true` to `src/cra/config/robot.yaml`.
  Set it to the YAML boolean `false` to run Simulation without Target GUI.
- Updated `src/cra/launch/simulation.launch.py` to read the installed robot.yaml
  and start `simulation/target_gui` alongside `simulation/robot_simulation` only
  when enabled. A missing setting defaults to disabled; nonboolean settings are
  rejected. Simulation itself runs in both enabled/disabled cases.
- Added a `workspace_path` launch argument and forwarded it from
  `src/cra/launch/cra.launch.py`, so GUI and Planning share the same workspace
  when the main launch uses an override. The standalone Simulation launch uses
  the same workspace default as the main launch.
- Declared `python3-yaml` in `src/cra/package.xml` for the launch-time YAML reader.
  No changes to Target GUI internals, Planning, Motion, or Vision were needed.

Build and run on the ROS workstation:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cra simulation --symlink-install
source install/setup.bash
ros2 launch cra simulation.launch.py
```

The existing `ros2 launch cra cra.launch.py` also starts the GUI according to
the same YAML switch. Both launch entry points accept
`workspace_path:=/absolute/path/to/raw_workspace.npz`.
The launch reads the installed `cra/config/robot.yaml`; rebuild `cra` after
editing the source config if the installed file is not symlinked to it.

Verification: Python syntax, Flake8, and package XML checks passed. Diff checks
preserved and accepted the existing CRLF line endings in the main launch and
manifest. Inline checks with real PyYAML and mocked ROS launch APIs passed for
enabled=true, enabled=false, missing switch, nonboolean rejection, and sharing
the same workspace argument between GUI and Planning. Live ROS launch, the
native GUI, and MuJoCo integration remain unverified in this environment.

### 2026-11-13 — CRA Target Control GUI

Files:
- Created `src/simulation/simulation/target_gui.py`: one ROS node and one Tk GUI
  class, workspace loading, linked mouse interaction, numeric preview editing,
  validation, and explicit target submission.
- Modified `src/simulation/setup.py`: registered the `target_gui` executable.
- Modified `src/simulation/package.xml`: declared `python3-matplotlib` and
  `python3-tk`; NumPy, rclpy, geometry_msgs, and visualization_msgs were already
  declared. No additional GUI framework or nearest-neighbor library is needed.
- Updated this status document with usage, evidence, and remaining checks.

Source/interface verification:
- `RobotSimulation.target_callback()` accepts
  `/simulation/set_target` (`geometry_msgs/msg/Point`) in CRA `base_link`.
  `update_control()` uses the existing `robot_base_to_world()` conversion.
- `/simulation/target_marker` is `visualization_msgs/msg/Marker` in `base_link`.
  The GUI matches the simulator's reliable, transient-local, depth-one marker
  QoS. Commands use the simulator's existing reliable, volatile, depth-ten QoS.
- `planning/workspace/generate_workspace.py` stores meter-valued `eof` samples
  after the same MuJoCo-base-to-CRA rotation used by Simulation's FrameTransform.
  The actual raw workspace has `coordinate_frame=base_link` and
  `source_coordinate_frame=mujoco_base`. Unexpected/missing frame metadata and
  malformed/nonfinite sample arrays are rejected rather than guessed.
- The default input matches `cra.launch.py`:
  `~/living-lab/cra_ws/src/planning/data/raw_workspace.npz`.
  The generator's separate default output at
  `~/.ros/cra_planning/raw_workspace.npz` was also inspected. Both contain
  50,000 EOF samples with the same bounds in this checkout.
- Actual EOF bounds, in meters (rounded here for reporting only):
  X `[-0.577250, 0.579945]`, Y `[-0.667035, 0.257217]`,
  Z `[-0.596251, 0.556359]`. Validation uses the unrounded extrema of all samples.
- Top View plots horizontal Y (+Y left) and vertical X (+X forward/up on the
  plot), consistent with the right-handed CRA base frame and its existing
  `(x, y, z) -> (x, -z, y)` conversion. Side View plots X horizontally and Z
  vertically (+Z up). No extra coordinate conversion is applied to commands.

Behavior:
- Both plots share one preview. Gray circles show Current Target and orange
  diamonds show Preview Target. Coordinates are displayed with three decimals;
  unchanged preview coordinates retain their underlying precision when sent.
- Select and drag the orange marker on either plot. Numeric coordinates can
  also be edited and committed with Enter or Apply Preview. SET TARGET applies
  pending numeric edits before validation. RESET copies the last valid feedback.
- Only SET TARGET publishes, once per click. No automatic retries or publishing
  occur during dragging, RESET, idle polling, or feedback processing.
- All samples determine bounds and nearest-sample distance; only the scatter
  display is reduced to a deterministic maximum of 12,000 points per projection.
  Four-percent plot padding is visual only. No filled reachable area is inferred.
- Nonfinite and out-of-bounds targets are blocked. An in-bounds target farther
  than 0.030 m from its nearest sample displays a warning but may be submitted.
  The threshold is a configurable visual heuristic, not a reachability test.
  Even nearby samples do not establish reachability or collision validity.
- The first valid feedback initializes the preview; subsequent feedback changes
  only Current Target. Wrong-frame, nonfinite, or non-ADD feedback blocks sending.
- Sending requires a matched command subscriber, a feedback publisher, and valid
  feedback received within two seconds. Callback processing uses nonblocking
  `rclpy.spin_once(timeout_sec=0.0)` on a Tk timer, without another thread.
- Published commands show "Target command sent" until later feedback matches
  the submitted position within 0.0001 m. This confirms the observed position,
  not delivery causality or successful planning. Confirmation times out after
  five seconds without resubmitting. Disconnected/stale feedback disables SET.

Usage on the ROS 2 Humble workstation, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select simulation --symlink-install
source install/setup.bash
ros2 run simulation target_gui --workspace "$PWD/src/planning/data/raw_workspace.npz"
```

Run the existing Simulation/Planning stack in another sourced terminal.
For a Simulation-only check, use `ros2 launch cra simulation.launch.py`.
If Planning was launched with a custom `workspace_path`, pass that same file
to the GUI's `--workspace` argument. Optional:
`--sample-warning-distance 0.03` changes only the visual proximity warning.
The GUI does not start or replace Planning or Motion.

Verification performed:
- Python compilation passed for the GUI and setup.py, with bytecode output
  directed to `/tmp` because pre-existing source __pycache__ is not writable.
- Package XML parsing, Flake8 (99-character limit), and `git diff --check` passed.
- Actual NPZ data and metadata were read and its bounds verified.
- A numerical check confirmed identical rotation matrices in Planning's generator
  and Simulation's FrameTransform, a proper orthogonal rotation, and a correct
  CRA-to-MuJoCo-base round trip. Static inspection also confirmed exactly one
  publish call, the executable registration, and required manifest dependencies.
- Temporary headless checks used real NumPy and Matplotlib Agg, with mocked Tk
  widgets and ROS communication. Both projections and sample plots were created;
  actual Matplotlib MouseEvents exercised marker selection, XY/XZ dragging,
  synchronization, cross-plot drag rejection, and release.
- Checks passed for RESET/numeric editing without publishing, preservation of
  preview precision, finite/bounds validation, nearest-sample warnings, invalid
  workspace/frame rejection, one publish per SET, feedback-only current updates,
  matching-feedback confirmation, stale/missing connection blocking, recovery,
  confirmation timeout without retries, and closing the GUI timer.
- NumPy/Matplotlib/Flake8 were installed only under `/tmp/cra-target-gui-deps`
  for these checks; the project's Python environment was not changed. The
  temporary harness is `/tmp/check_cra_target_gui.py`, outside the repository.

Remaining limitations/checks:
- This execution environment has no ROS 2 installation, colcon, Tkinter, or
  MuJoCo. A native GUI launch, ROS package build, DDS QoS matching, target motion
  in MuJoCo, and Planning's response have not been run.
- Verify the real Tk window and mouse/numeric controls on the workstation, then
  submit several targets while observing Simulation feedback and the existing
  Planning/Motion flow. Confirm there are no commands during preview operations.
- This GUI is not verified end-to-end. No changes were made to Planning, GNG,
  A*, CollisionFilter, PlanningManager, Motion, Vision, generated workspace data,
  or the experiment settings `execute_path=True` and
  `simulation_only_ignore_unknown=True` during this task.

### 2026-11-13
Human Obstacle Avoidance Debugging

Problem:
Path planning frequently fails after
Vision and Planning integration.

Investigation identified several possible causes:

- Large human capsule radius
- Unstable radius estimation
- Conservative robot collision representation
- Possible coordinate-frame inconsistencies
- Frequent environment updates caused by
  small changes in human geometry

Proposed and prepared changes:

1. collision_filter.py
   - Improve human collision calculations.
   - Improve collision diagnostics.
   - Review coordinate transformations.
   - Preserve existing collision interfaces.

2. human_collision.py
   - Separate human-specific collision calculations.
   - Simplify collision_filter.py.

3. human_capsule.cpp
   - Improve radius percentile estimation.
   - Preserve configured radius limits.
   - Maintain existing capsule representation.

4. reconstruction_pipeline.cpp
   - Adjust temporal radius smoothing.
   - Reduce immediate expansion caused by
     isolated large measurements.
   - Preserve previous radius when measurement
     quality is insufficient.

5. reconstruction.yaml
   - Proposed radius_percentile: 0.80
   - Proposed min_voxels: 15
   - Keep radius_smoothing_alpha: 0.20
   - Original proposal: capsule safety_margin 0.015; superseded by the
     authorized CRA setting 0.0 described in the latest development entry.

Important:
These changes were prepared during ChatGPT development.

Their application to the active local repository
has not been independently confirmed.

Do not mark these changes as fully implemented
until the repository has been inspected.

## 6. Current Experiment Configuration

Planning:

execute_path: true
simulation_only_ignore_unknown: true

Vision capsule extra padding: 0.0 m
Planning human separation margin: 0.10 m
Positive anatomical and measured human capsule radii remain active.

These are user-selected experimental settings.

Do not modify them without an explicit request.

Important:
Unknown human data may be ignored in this
simulation configuration.

Successful human avoidance must be evaluated
using valid human geometry data.

## 7. Known Technical Issues

### Issue 1: Path Planning Failure

Problem:
Planning frequently fails when human obstacles
are enabled.

Possible causes:
- Excessive obstacle geometry
- Invalid start or goal configuration
- Conservative collision representation
- Limited graph connectivity
- Frequent environment changes

Status:
Under investigation.

Next action:
Inspect Planning failure status and
collision diagnostics.

### Issue 2: Human Capsule Size

Problem:
Human capsule geometry may be excessively large.

Possible causes:
- Default radius constraints
- Voxel-based radius estimation
- Noisy depth measurements
- Temporal filtering behavior

Status:
Code adjustments prepared.

Next action:
Compare capsule geometry against skeleton
and voxel data in RViz.

### Issue 3: Human Capsule Instability

Problem:
Capsule radius changes between frames.

Possible causes:
- Depth measurement noise
- Skeleton fluctuations
- Voxel assignment variations
- Radius smoothing behavior

Status:
Smoothing adjustment prepared.

Next action:
Test stationary and moving human scenarios.

### Issue 4: Coordinate Frame Consistency

Problem:
Vision and Planning geometry may use
different coordinate transformations.

Status:
Requires verification.

Next action:
Validate Vision base_link coordinates
against MuJoCo robot geometry.

### Issue 5: Conservative Collision Geometry

Problem:
Planning uses bounding sphere approximations
for robot collision geometry during human
collision checking.

Status:
Existing implementation identified.

Next action:
Evaluate collision accuracy after
coordinate-frame verification.

### Issue 6: Dynamic Environment Updates

Problem:
Small human geometry changes may cause
frequent environment_revision updates.

Status:
Requires evaluation.

Next action:
Inspect obstacle update frequency
and path revalidation behavior.

## 8. Verification Status

Previously reported isolated checks:
- Python syntax checks
- Basic human capsule validation logic
- Basic C++ geometry tests using mock inputs

These checks do not establish successful
ROS 2 or MuJoCo integration.

Not yet verified after recent changes:
- Complete ROS 2 build
- Vision topic communication
- Coordinate-frame consistency
- Human collision accuracy
- GNG and A* planning success rate
- Dynamic path revalidation
- Motion stop and replanning behavior
- End-to-end MuJoCo experiments

No end-to-end success claim should be made
until relevant experiments have been completed.

## 9. Next Development Tasks

Priority 1:
Inspect the actual local repository and
verify which recent changes are installed.

Priority 2:
Build the modified Vision and Planning packages.

Priority 3:
Verify coordinate transformations between
Vision and MuJoCo.

Priority 4:
Evaluate capsule radius and stability in RViz.

Priority 5:
Inspect collision reports and Planning failures.

Priority 6:
Evaluate path planning with static human geometry.

Priority 7:
Evaluate dynamic obstacle avoidance
with moving human geometry.

Priority 8:
Measure planning performance and path success rate.

## 10. Development Rules

- Preserve GNG and A*.
- Use src/planning/ as the active Planning package.
- Do not modify workspace_planning.
- Avoid unnecessary code restructuring.
- Maintain ROS 2 interface compatibility.
- Verify source code before changes.
- Document actual test results.
- Keep implementation and verification status separate.

## 11. Next Codex Task

Read:
- AGENTS.md
- CRA_PROJECT_CONTEXT.json
- CRA_PROJECT_STATUS.md

Inspect the actual repository.

Focus on:
- collision_filter.py
- human_collision.py
- human_capsule.cpp
- reconstruction_pipeline.cpp
- reconstruction.yaml

Verify:
1. Which proposed changes are actually present.
2. Whether public interfaces remain compatible.
3. Whether coordinate transformations are correct.
4. Whether parameters are loaded correctly.
5. Whether there are implementation regressions.

Do not modify source code during the initial review.

Report:
- Current implementation
- Missing changes
- Potential problems
- Verification results
- Recommended next steps
