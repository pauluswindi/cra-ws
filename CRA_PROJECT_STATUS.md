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
- Once a close human has triggered this recovery context, UNKNOWN/EMPTY cannot
  be used to execute a blind escape. STOP remains latched until valid observed
  geometry returns. Ordinary simulation behavior outside this context is
  unchanged; `simulation_only_ignore_unknown=True` and `execute_path=True` remain
  unchanged. This narrower recovery rule deliberately requires fresh geometry.
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
   - Keep capsule safety_margin: 0.015

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
