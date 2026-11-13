# Cognitive Robotic Arm (CRA)
# Development Status

Last updated: 2026-11-13

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
