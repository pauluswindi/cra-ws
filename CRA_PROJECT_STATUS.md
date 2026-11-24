# Cognitive Robotic Arm (CRA)
# Development Status

Last updated: 2026-11-24

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
    +-- RRT-Connect fallback (simulation only)
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
Simulation can try RRT-Connect after eligible graph-search failure;
it uses the same validation and Motion execution pipeline.

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

### 2026-11-24 — Refresh stale inputs before admitting a failed graph's RRT attempt

After known NO_GRAPH_PATH, transiently stale feedback/obstacles now retain one
lightweight attempt with its target generation, start, goal and fixed deadline.
Planning returns RRT_WAITING_FOR_FEEDBACK/RRT_WAITING_FOR_OBSTACLES so input
callbacks can run; it does not immediately repeat A* or launch a worker yet.
Fresh input triggers start/goal collision checks on the latest human scene,
then the existing RRT coordinator and final Motion validation. Waiting consumes
the existing fallback allowance and never extends its deadline.

Changed target, moved start, static obstacle changes, faults, external STOP,
active recovery, unknown/expired human data and timeout cancel the saved
attempt. Human proximity checks still run first; HUMAN_TOO_CLOSE cancels the
attempt and retains Human Retreat priority. Profiles now report
rrt_admission_reason and rrt_refresh_pending, with cancellation reasons in
rrt_result. Camera configuration, YAML, 1 s feedback/obstacle freshness, 2 s
human freshness, 5 cm margin, execute_path and simulation UNKNOWN policy are
unchanged. A* remains synchronous; this does not bound its own callback time.

Verification: 149 fallback/dynamic-replanning tests and Flake8 passed; Planning
was built and installed. Native regression delayed the first real A* call by
1100 ms on the temporary disconnected graph: RRT_WAITING_FOR_FEEDBACK ->
RRT_SEARCHING -> RRT_PATH_READY -> source=RRT_CONNECT, then Motion COMPLETED.
Exactly one primary plan call occurred before the first RRT goal completed;
maximum measured joint error was 0.0197165 rad. Return motion stopped after
synthetic geometry became invalid, with no further EXECUTE/RELEASE in the
probe's following 0.5 s window. Source hashes matched the installed run.
This uses real Planning/C++ Motion/headless MuJoCo and synthetic human input;
live Kinect/moving-human validation of this handoff remains unverified.

Evidence: `src/planning/test/results/rrt_input_refresh_simulation_20261124.json`
and `src/planning/test/results/rrt_input_refresh_simulation_20261124.log`.

### 2026-11-24 — Bounded latest-only log on normal CRA launch

Normal CRA launch now records child stdout/stderr and process start/exit
metadata to the YAML-configured `logs/cra_latest.log`. It locks the file before
truncation at launch execution, replaces only its own file, and leaves ROS,
colcon and unrelated logs untouched. Description loading/--show-args has no
file side effect. The new `logs/` path avoids the non-writable root-owned
colcon log directory for host UID 1000 and is ignored by Git.

ProcessIO callbacks only enqueue bounded 8 KiB chunks into a 256-entry queue.
A daemon writer frames lines, removes ANSI color, uses buffered I/O and flushes
approximately every 0.5 s. Queue overflow drops chunks with an explicit counter;
the file truncates in place at the configured 20 MiB cap to retain newer output.
CLI atexit drains teardown output; shutdown join is bounded to 2 s. Disk errors
disable the added capture while console/control processes continue. The
existing console-only demo explicitly bypasses the additional logger.

Verification: nine logger unit cases and ten launch/data-path regressions
passed, Flake8 passed, CRA built and installed. A hardware-free native ROS
probe used C++ TF and Python subprocesses for two consecutive normal launches,
plus disabled and console-only cases: overwrite, stdout/stderr, fragmented
Unicode, process exit and final shutdown output passed. Existing native demo
console tests also passed. No full CRA/camera/Motion run or whole-pipeline
performance measurement was performed; sensor, planning and control settings
are unchanged. Evidence: `src/cra/test/results/latest_log_20261124.json`.
Usage and limits: `src/cra/LATEST_LOG.md`.

### 2026-11-24 — RRT fallback executed through native Motion and MuJoCo

Ran `verify_rrt_fallback_simulation.py` in isolated localhost domain 228.
Its temporary two-node, zero-edge roadmap makes A* unable to connect the
start and goal; the original workspace graph and YAML were not modified.
Planning submitted and validated two RRT attempts and dispatched two
three-waypoint trajectories with `source=RRT_CONNECT`. The first reached
Motion COMPLETED with maximum measured joint error 0.0196097 rad; the first
goal sequence took 4.0216 s, including planning and execution.

During the return trajectory, synthetic HumanCapsuleArray input was changed
to valid=false. Planning reported RRT_HUMAN_VISION_UNKNOWN and native Motion
reported STOPPED. No additional trajectory or RELEASE was observed during
the probe's subsequent 0.5 s observation window. The report's legacy
camera_loss fields refer to this synthetic invalid-input injection, not an
actual Kinect disconnection. Source hashes matched the run's files.

This verifies real Planning/C++ Motion/headless MuJoCo execution of RRT
fallback. The synthetic human was far away and the RRT edge was directly
valid (two root markers, no random detour growth). It does not verify live
Kinect avoidance, physical braking or routing around a moving human.
Evidence: `src/planning/test/results/rrt_usage_simulation_20261124.json` and
`src/planning/test/results/rrt_usage_simulation_20261124.log`.

### 2026-11-23 — YAML defaults for RViz and tracker admission

At user request, robot.yaml now contains `rviz.enabled: true` and
`vision.tracker_enqueue_wait_ms: 5`. CRA launch reads both defaults; the CRA
Vision entrypoint reads the same tracker setting and forwards it to the
driver. Explicit CLI overrides remain available. Missing fields retain the
previous fallbacks of false/-1, and invalid YAML types or unsupported waits
are rejected. Sensor FPS/resolution/mode, smoothing, calibration and safety
parameters are unchanged. The standalone driver's own default remains -1.

Ten Python launch/data-path tests and Flake8 passed. CRA was rebuilt with
symlink installation. Installed launch --show-args confirms defaults of
rviz=true and tracker_enqueue_wait_ms=5 without starting any nodes. No camera
or manual experiment was run. Normal startup now needs only
`ros2 launch cra cra.launch.py` in the sourced container.

### 2026-11-23 — Installed tracker admission option, blocking by default

The installed Kinect driver now accepts read-only startup ROS parameter
`tracker_enqueue_wait_ms`: -1 retains blocking admission, 5 uses the measured
bounded-wait policy. Main CRA, Vision and body-tracking launches declare and
forward the option as an integer; omitting it keeps blocking. A bounded SDK
timeout skips only that tracker submission, without adding a pending result;
SDK FAILED still propagates to the existing worker shutdown handling.
No automatic mode switching or live parameter changes were added. The 0/10 ms
variants remain restricted to the BUILD_TESTING executable. The comparison
runner now uses the installed driver for blocking and 5 ms and records the
driver variant/hash.

Fixed the runtime install destination to `lib/azure_kinect_ros_driver/node`.
The previous generic bin destination left ROS launching a stale lib copy
despite successful builds. The installed executable now matches the build
binary and reports the selected admission policy before opening a device.

Verification: azure_kinect_ros_driver/cra/vision built and installed; 17 C++
stream-control tests, 11 native startup cases with a camera-denying shim,
and 28 Python launch/data-path/runner/scenario cases passed. Native startup
also passed against the installed executable. Launch --show-args exposes
the option. Camera/workspace/calibration values and robot/reconstruction
YAML hashes match the preceding participant trial. No new manual experiment
or live Kinect/MuJoCo run was performed after promotion; previous live
evidence remains scoped to its recorded binaries. Human obstruction during
target execution remains unverified from the last not_exercised trial.

Use `ros2 launch cra cra.launch.py rviz:=true tracker_enqueue_wait_ms:=5`
inside the sourced container, or restart with -1 for blocking. Instructions:
`src/azure_kinect_ros_driver/TRACKER_ADMISSION.md`. Evidence:
`src/azure_kinect_ros_driver/test/results/tracker_main_integration_20261123.json`.

### 2026-11-23 — Trigger disturbance tests from target Motion execution

The tracker comparison now accepts `--scenario vision-loss` or `blockage`
with the existing isolated MuJoCo mode. It waits for recent observed human
geometry and IDLE/COMPLETED after warm-up, sends one guarded target, and
triggers the disturbance cue only on that target's EXECUTING status. Initial
simulator motion cannot activate it. The original four timed phases remain
available without `--scenario`.

Both scenarios pass depth through the native probe to Reconstruction only.
Vision-loss withholds that input for 3 s, then restores forwarding; the
camera driver is not paused or reconfigured. Blockage requires recorded
HUMAN_TOO_CLOSE/HUMAN_PATH_BLOCKED evidence while the target is executing.
Reports classify passed/failed/not_exercised and preserve event times.
Completion before disturbance is not a pass. Passing recovery requires STOP,
STOPPED, observed geometry, RELEASE, a new trajectory, EXECUTING and both
Motion/Planning completion. DDS boundary ambiguity is marked for review;
the evaluator's 0.2 s receipt uncertainty changes no runtime timeout.

Verification: 20 Python regression cases and Flake8 passed; the C++ probe
built and its original four-phase CLI/JSON was verified. A synthetic DDS
fixture exercises the actual depth relay and Reconstruction watchdog. A
separate synthetic-input test ran real Planning, C++ Motion and headless
MuJoCo: depth withholding at 6.209 s, invalid geometry at 6.717 s, STOPPED
at 6.795 s, restored depth at 9.210 s, fresh observed geometry at 9.256 s,
resumed execution at 9.605 s and Motion completion at 10.868 s. Times are
observer receipt times, not end-to-end reaction guarantees.

No Kinect/live-human disturbance trial has been run for this new scenario.
Physical braking distance, pose accuracy, USB disconnect/restart and RRT
fallback are not established by these tests. No production camera/YAML,
Planning/Motion behavior or legacy planner changed. Normal launch remains
blocking. Depth relay overhead means these scenario rates must not be treated
as the original direct-pipeline FPS comparison.

Instructions: `src/perception/vision/test/TRACKER_MOTION_SCENARIOS.md`.
Evidence: `src/perception/vision/test/results/tracker_motion_scenarios_20261123.json`.

### 2026-11-23 — Camera startup restored with the host user UID

After USB detection recovered to one sensor, the integration driver failed
with Depth Engine error 204. The complete log first showed X11 authorization
failure, including during Transform Engine initialization. A direct X11
connection failed as container root but succeeded as host UID 1000. NVIDIA
devices and `nvidia-smi` were available; GPU enumeration alone did not prove
display access.

A 20 s real-camera-only probe under UID/GID 1000:1000 received 184 depth,
124 RGB and 334 body-tracking messages, with zero capture timeout warnings
and completed cleanup. Original camera settings were preserved. Report:
`/tmp/cra_kinect_uid1000_probe_20261123.json` inside `cra-humble`.
This verifies camera startup and stream delivery, not the controlled MuJoCo
comparison. Run the latter with `docker exec -u "$(id -u):$(id -g)"`,
`bash -c`, a writable `ROS_LOG_DIR`, and a fresh output directory to avoid
the previous root-owned trial files. No display access rules were changed.

### 2026-11-23 — Reject tracker trials when the camera is unavailable

Both modes in `/tmp/cra_tracker_mujoco` reported `Found 0 sensors` and
failed to start cameras. Blocking recorded 40 s invalid geometry and no
camera streams; the requested target was skipped. The 5 ms assertion was
secondary: its admission log is emitted only after camera startup. These
trials provide no tracker performance comparison. A live SDK enumeration
also returned zero sensors; the container USB inventory showed no Kinect.
The physical connection or power cause remains unverified.

The comparison harness now checks SDK sensor enumeration before starting
any trial, and monitors required child processes during both guided and
unguided runs. A driver that exits with code zero still aborts the trial
with its log tail. Camera settings, YAML and Planning/Motion are unchanged.
Six isolated Python regression tests and Flake8 passed. Running the real
integration command with the camera absent rejected it in 0.08 s without
creating trial output or starting simulation. Restore Kinect detection,
then repeat the controlled blocking/5 ms MuJoCo comparison.

### 2026-11-20 — Guided tracker movement and occlusion comparison

The Vision-only comparison supports `--guided` for stillness, repetitive
arm motion, partial arm occlusion by the torso, and recovery. It runs the
same cues for both modes and measures geometry state duration, stream rates,
observation ages, and longest silence per phase. A phase with no skeleton
messages records its full duration as silence. Participant actions must
follow the cues for a controlled comparison.

Run this in one terminal from the sourced CRA container with the regular
camera launch closed:

```bash
source /opt/ros/humble/setup.bash
source install/msgs/share/msgs/local_setup.bash
source install/vision/share/vision/local_setup.bash
source install/azure_kinect_ros_driver/share/azure_kinect_ros_driver/local_setup.bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  python3 src/perception/vision/test/compare_tracker_admission.py \
  --guided --modes blocking wait-5ms --warmup 15 --duration 40 \
  --output /tmp/cra_tracker_motion
```

The sampler writes phase metrics to `comparison.json` in the output
directory. Four timed cues, phase accounting, invalid geometry, missing
skeleton messages, and recovery were verified with synthetic DDS inputs.
The 5 ms Kinect smoke run completed all four cues and wrote the report.
The later participant run completed all four phases in both blocking and
5 ms with a person detected in the workspace throughout both 40 s windows.
Both trials reported 40 s observed geometry and no invalid state during
measurement. The camera reported no capture timeouts. The invalidation
logs after each run followed camera shutdown.

Blocking versus 5 ms: median skeleton age 318 vs 88 ms, skeleton rate
25.5 vs 16.6 Hz, RGB receipt rate 10.0 vs 14.8 Hz, and observed geometry rate
14.5 vs 18.7 Hz. During arm motion, the longest observed geometry message
gap was 432 vs 501 ms. During partial occlusion it was 399 vs 270 ms.
The 5 ms mode is a candidate for isolated MuJoCo integration. This single
pair measures output continuity and timing with the participant cues.
Pose accuracy during occlusion and response latency through Planning and
Motion require separate evaluation.

Results: `src/perception/vision/test/results/tracker_admission_guided_20261120.json`.

The guided runner also supports an isolated MuJoCo integration trial with
`--mujoco --target 0.50 0.10 -0.25`. The installed configuration must specify
simulation mode. The runner starts MuJoCo, Planning and Motion alongside
Vision and sends the test target at the arm-motion cue when the monitor has
recent observed geometry in the workspace. Each mode starts its own instance
of the simulator. Physics runs headless by default. Use `--viewer` for the
interactive MuJoCo window.

The report includes `simulation_events` for target, Vision, Planning and
Motion status, STOP/RELEASE, and trajectory messages. Event times are DDS
receipt times. Closely spaced events across topics can be received in either
order. The trial uses the regular simulated motion policies. Record any live
camera invalidation that occurs during an active trajectory to evaluate the
stop response on that run.

### 2026-11-20 — Bounded tracker enqueue comparison

The test-only tracker binary accepts a 0, 5 or 10 ms SDK enqueue wait.
The normal CRA launch uses the installed blocking driver. The real-camera
comparison used the existing calibration and reconstruction settings in
isolated domain 221 with Vision nodes only.

Seven 25 s measurement windows followed 15 s warm-ups. A human was active in
the zone and geometry was continuously in the observed state in all measured
windows. The three experimental modes were repeated in reverse order.
The 5 ms mode produced 15.7–16.0 skeleton updates/s, with median data age
80–86 ms and maximum observed callback gap 165 ms. The 10 ms mode produced
17.5–17.7 updates/s with median age 87 ms and a maximum observed gap of 294 ms.
The 0 ms mode produced 14.6 updates/s and the highest received RGB rate.
The 5 ms mode is the candidate for a controlled movement and occlusion trial.
This run measured stream behavior. Pose accuracy and camera-loss recovery
require separate trials.

Build the experimental target and run the series inside the sourced CRA
container with the regular camera launch closed:

```bash
cmake --build build/azure_kinect_ros_driver --target tracker_nonblocking_probe -j2
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  python3 src/perception/vision/test/compare_tracker_admission.py \
  --modes blocking wait-5ms wait-10ms nonblocking --output /tmp/cra_tracker_wait
```

Results and source hashes are in
`src/perception/vision/test/results/tracker_admission_bounded_wait_20261120.json`.

### 2026-11-20 — Isolated nonblocking tracker A/B and B/A experiment

Status: User-authorized experiment complete; normal launch still BLOCKING.
Nonblocking is a BUILD_TESTING-only `tracker_nonblocking_probe` executable,
excluded from default builds and never installed. SDK TIMEOUT skips only the
tracker submission; accepted work alone increments the pending-result counter.
SDK FAILED exceptions still propagate through the existing worker handler.

- Real Kinect, original camera parameters, TF calibration, workspace monitor,
  skeleton transform and reconstruction YAML on localhost domain 221. No GUI,
  Planning or Motion. A native subscriber records headers/counters only; no
  images, skeleton coordinates or capsule payloads are saved.
- Four trials: blocking/nonblocking, then nonblocking/blocking. Each uses
  15 s warm-up plus 30 s measurement. A real person is active in the original
  workspace throughout all 120 measured seconds. All pipeline children remain
  alive; capture cleanup completes and no fatal capture timeout occurs.
- Repeated ranges, blocking -> nonblocking: SDK capture consumption about
  24.7-25.0 -> 30.0 Hz; received RGB 8.4-9.9 -> 17.2-19.1 Hz; observed geometry
  11.4-11.7 -> 18.2-20.4 Hz. Median skeleton capture age 318-322 -> 80-83 ms;
  median geometry depth-stamp age 241 -> 30-31 ms. No invalid-state interval
  is observed during the warmed measurement windows in either mode.
- Important tradeoff: skeleton update rate drops 24.5-25.1 -> 14.7-14.8 Hz.
  Geometry can reuse a pose within the existing sync allowance, so geometry Hz
  must not be presented as independent pose-measurement Hz. Faster/fresher
  geometry does not establish fast-motion tracking accuracy or execution safety.
- 15 driver policy/shutdown C++ cases and four native startup cases pass.
  Busy submission creates no pending result; real failures still propagate.
  Python style, whitespace and preserved-setting/source hashes pass.
- Production mode is intentionally unchanged. Controlled fast-person motion,
  occlusion, long capture stability and full CRA operation remain unverified.
  No deadline, max_sync_delay, freshness, smoothing, calibration, FPS/resolution,
  Planning/Motion policy or stored workspace asset was changed.

Reproduce inside the sourced container/workspace after closing the normal
camera launch:

```bash
cmake --build build/azure_kinect_ros_driver --target tracker_nonblocking_probe -j2
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
  python3 src/perception/vision/test/compare_tracker_admission.py \
  --output /tmp/cra_tracker_comparison
```

Files: src/azure_kinect_ros_driver/{CMakeLists.txt,src/k4a_ros_device.cpp,
include/azure_kinect_ros_driver/stream_control.h,test/test_stream_control.cpp};
src/perception/vision/{CMakeLists.txt,test/vision_stream_probe.cpp,
test/compare_tracker_admission.py}.
Results: src/perception/vision/test/results/tracker_admission_ab_20261120.json.

### 2026-11-20 — Camera-loss/FPS diagnosis; nonblocking tracker still proposed

The new supplied log contains a real five-timeout SDK stream shutdown while
Planning is COMPLETED, followed by respawn and recovery. A later 0.200 s
VISION_INVALID event has no SDK timeout in the excerpt. RGB/depth are requested
as synchronized pairs; continuing Color queue drops do not establish the
original reason a synchronized capture stopped arriving.

- Applied preview-only native reduced JPEG decode for the existing half-size
  view. Original JPEG/depth inputs, YAML, 30 FPS/1536P, smoothing, calibration,
  synchronization and data timeouts remain unchanged. Preview pixel values may
  differ slightly from the previous full-decode/resize, with the same size.
- Previously empty Vision build type now defaults to RelWithDebInfo (-O2 and
  debug symbols), honoring explicitly chosen build modes. All existing explicit
  geometry checks remain. Direct image callback decode is retained: a tested
  timer-decode experiment lowered view cadence and was discarded.
- Separate Input/View FPS in the GUI, plus five-second Preview profile and
  Capture profile timing. Reconstruction invalid diagnostics report input ages,
  sync delta and last blocking reason. These counters never refresh observations
  or change timeout/retry behavior.
- Controlled JPEG decode/resize median improves 16.52->12.57 ms. Seventeen
  Vision C++ cases, 12 driver C++ cases and four native startup cases pass.
  Native reconstruction preserves eight capsules, stale invalidation and fresh
  EMPTY. Synthetic headless preview verifies raw priority, compressed fallback
  and survival of input loss; it does not establish live GUI/full CRA FPS.
- Crucial real-camera measurement at unchanged settings: complete SDK capture
  consumption about 24-25 Hz, get_capture wait about 0.003 ms, processing about
  33-35 ms and tracker enqueue commonly 30-42 ms. enqueue_capture currently
  waits indefinitely in the same thread that publishes RGB/depth. The isolated
  60 s run has no capture timeout; original intermittent loss is not fixed.
- Proposed, NOT applied: zero-timeout tracker enqueue; skip that tracker input
  when busy, continue actual RGB/depth publication, retain original body result
  timestamps and all freshness policies. SDK FAILED remains fatal. User asked
  for pros/cons before deciding; queue behavior is unchanged pending decision.

Files: src/perception/vision/{CMakeLists.txt,src/nodes/{visual_node,
reconstruction_node}.cpp,include/vision/frame/preview_decode.hpp,
test/{test_preview_decode.cpp,benchmark_preview_decode.cpp,verify_preview_ros.py}};
src/azure_kinect_ros_driver/src/k4a_ros_device.cpp.
Evidence: src/perception/vision/test/results/camera_fps_diagnosis_20261120.json.

### 2026-11-20 — Vision optimization with unchanged YAML and camera settings

Status: User-authorized code optimization built and tested. No YAML, camera
launch settings, smoothing, calibration, voxel size, capsule parameters, sync
allowance or freshness timeout changed. Driver, Planning/Motion and RRT
implementation are untouched by this optimization.

- Replace the intermediate full-size per-body binary mask with ordered runs
  containing every matching body-index pixel. Depth projection visits those
  pixels in the original row-major order. No sampling, geometric approximation
  or dropped body pixels is introduced. The original mask-based C++ entry
  point remains available. Dimension, frame, encoding, transform, calibration,
  bounds and byte-order checks remain; malformed runs are rejected.
- VisualNode captures one TF snapshot for a skeleton batch instead of looking
  up the same transform for every joint. Missing TF suppresses that projected
  overlay; it no longer substitutes untransformed points. Geometry sent to
  Planning uses its unchanged frame/validation flow.
- Thirteen C++ cases pass: exact points for integer/float depth, padded rows,
  sparse/dense/fragmented masks, distortion/rotation/ROI and changed calibration;
  invalid inputs, exact voxels/capsules and 96 batch/per-joint TF comparisons.
  Installed reconstruction DDS probe on domain 231 produces eight base_link
  capsules, invalidates after input loss (observed 0.569 s under the existing
  0.5 s timeout/100 ms watchdog), then accepts fresh EMPTY. No camera or Motion
  runs in this probe. Vision build and probe Flake8 checks pass.
- Independent original committed DepthProcessor comparison on synthetic
  512x512 images requires exactly identical point vectors. With 35% selected
  pixels, selection/extraction median falls from 7.305 to 5.669 ms (22.4%).
  This stage improves 18.2-32.6% across tested nonempty densities; whole Vision
  FPS and live CRA stability are not established. Mask shape and hardware can
  change the gain. Existing compiler optimization level is unchanged.
- Live Kinect/full CRA/moving-human operation and interactive GUI appearance
  remain unverified for this optimization. Previous intermittent capture
  failure is not declared fixed. Restart the launch to load the new binaries.

Files: src/perception/vision/{src/segmentation/human_segmenter.cpp,
src/reconstruction/{depth_processor,reconstruction_pipeline}.cpp,
src/nodes/visual_node.cpp,include/vision/{segmentation,reconstruction,frame},
test/{test_sparse_depth.cpp,test_frame_batch.cpp,vision_fixture.hpp,
benchmark_sparse_depth.cpp,verify_reconstruction_ros.py},CMakeLists.txt,package.xml}.
Results and source/configuration hashes:
src/perception/vision/test/results/vision_optimization_20261120.json.

### 2026-11-20 — Retreat dead-end correction and unchanged-camera diagnosis

The supplied 23:46 runtime log loads rrt_mode=fallback but has 85 profiles
with NOT_TRIED, six A* PATH_FOUND and no NO_GRAPH_PATH. Its long stop is
HUMAN_RETREAT_BLOCKED, including 0.01812 m and 0.00747 m current clearance
under the existing 0.05 m margin. Ordinary RRT is not admitted from that start.
SEARCH_BUDGET_EXHAUSTED is the retreat's 32-step bound, not a wall deadline;
logged retreat searches average 273.0 ms and peak at 439.786 ms.

- Rank improving endpoints before transition certification. The same
  highest-gain certified candidate wins, with stable joint/sign ties; lower
  candidates need no edge checks after a better one passes.
- At a local dead end, consider the same staged-progress certificate already
  used at the step limit instead of discarding progress immediately. Minimum
  gain remains 0.005 m; complete straight-transition certification and all
  normal Manager/RELEASE/final execution gates remain mandatory. No contacts,
  pair-clearance relaxation, worker or ordinary RRT invalid-start escape added.
- Add bounded retreat statistics to the existing Planning profiles: completed
  steps, endpoint/transition queries, clearance/gain, final certificate verdict,
  stop condition and elapsed time. Search remains synchronous and greedy;
  blocked does not prove that no multi-joint escape exists.
- User explicitly retained camera settings. No FPS, resolution, smoothing,
  calibration, retry count, driver implementation or freshness limit changed.
  First capture timeout precedes heavy retreat; color queue drops begin after
  that first timeout. Exact physical/system cause remains unverified.
- Add a real-camera stream diagnostic on isolated domain 229, using parameters
  verbatim from the existing launch, without Planning/Motion or saved images.
  Native 25 s and 60 s runs have no capture timeout or SDK color queue drop,
  and complete worker shutdown. The 60 s probe receives 1375 skeleton messages
  (maximum measured capture age 421.84 ms). These received-message statistics
  do not establish sensor FPS, full CRA stability or a camera repair.
- Verification and final source hashes: test/results/retreat_debug_20261120.json.
  All 296 Planning tests pass in 29.836 s, including 10 new retreat cases;
  Planning build passes in 0.98 s, with Flake8 and whitespace checks passing.
  Twelve frozen native scenes retain the same 10 successful paths; transition
  calls fall from 346 to 228. Two scenes remain blocked. Installed-package
  RRT/Motion/MuJoCo regression again reaches its joint goal and stops on
  synthetic camera invalidation; this is not a live-human retreat trial.
  Offline native comparisons use synthetic capsules, including a logged joint
  pose; the original log lacks full geometry for exact failure replay.

Files: src/planning/planning/{path_planner.py,planning_node.py};
src/planning/test/{test_human_retreat_search.py,benchmark_human_retreat.py,
HUMAN_RETREAT_DEBUG.md};
src/azure_kinect_ros_driver/test/probe_capture_stream.py;
src/planning/test/results/retreat_debug_20261120.json; CRA_PROJECT_CONTEXT.json.

### 2026-11-19 — Active simulation RRT-Connect fallback

Status: User-authorized implementation, build, regression and isolated native
Planning -> C++ Motion -> MuJoCo verification passed. This supersedes the
observation-only runtime status in the older entries below; shadow mode remains
available.

- GNG+A* and configured alternative joint goals remain primary. Eligible
  NO_GRAPH_PATH requests one bounded RRT attempt before existing safe-position
  recovery. Invalid starts, human-margin intrusion, unreachable goals, manual
  STOP and Motion faults retain their existing handling.
- One isolated worker searches with ordinary CollisionFilter checks, then
  prepares/certifies the candidate on a new scene. A private mailbox delivers
  it to the regular PlanningManager step; normal path preparation, final
  validation and RELEASE acknowledgement still precede EXECUTE. Target/start/
  scene changes, expired inputs/deadlines and malformed results reject it.
- Fresh OBSERVED or EMPTY human input is mandatory. UNKNOWN is never accepted
  by fallback; camera loss during an active RRT trajectory requests STOP.
  Existing primary simulation_only_ignore_unknown=True and execute_path=True
  remain unchanged, as do the 0.05 m margin and 2 s observation timeout.
- YAML planning.rrt.mode defaults to fallback; off and shadow are available.
  Fallback is rejected outside simulation before node/model creation. Search
  allowance is 0.5 s, latest validation 0.5 s, total request 1.5 s including
  startup/handoff, capped by original data freshness; retries are >=3 s apart.
  Fallback omits the extra clearance-preference search used by shadow mode.
- RViz is optional; no marker subscriber means no trace/FK overhead. Events on
  /planning/rrt/events and legacy /planning/rrt_shadow explain admission and
  rejection. RRT_PATH_READY and source=RRT_CONNECT identify adopted paths.
  Trees are expiring diagnostics; normal Planned path shows the manager path.
- Verification: 286/286 Planning tests (38 fallback tests and 19 RRT marker
  tests) pass in 30.006 s; 5 launch-default tests pass. planning/cra build,
  Flake8 and whitespace checks pass. CollisionFilter, PathPlanner, Motion,
  MuJoCo model/control and legacy workspace_planning are unchanged.
- Native domain 228 uses the installed packages, real C++ Motion and actual
  MuJoCo physics/control/feedback with a headless viewer and synthetic vision.
  On a temporary two-node/zero-edge roadmap, RRT reaches the first joint goal:
  maximum joint error 0.01948 rad in 4.020 s, Motion COMPLETED. A return
  trajectory followed by invalid camera input produces STOPPED and no further
  EXECUTE/RELEASE while invalid. Start/goal/final markers are received.
  Separate shadow-mode DDS probe passes with zero Motion commands.
- Limits: native execution uses a deliberately disconnected small roadmap and
  a directly free RRT edge. Live Kinect, moving-human detours, production-graph
  failure, interactive RViz and physical execution remain unverified. Existing
  A*/human monitoring/final execution checks remain synchronous; cooperative
  deadlines reject late results but cannot interrupt native collision calls.

Built configuration is installed. Restart the existing launch, then use
`ros2 launch cra cra.launch.py rviz:=true`; dataset paths come from YAML.

Files: src/planning/planning/{rrt_fallback.py,rrt_shadow.py,planning_manager.py,
planning_node.py,visualization/rrt_display.py}; src/cra/config/robot.yaml;
src/planning/test/{test_rrt_fallback.py,verify_rrt_fallback_simulation.py,
test_rrt_visualization.py,verify_rrt_shadow_ros.py,RRT_FALLBACK.md,
RRT_SHADOW.md,RRT_VISUALIZATION.md};
src/planning/test/results/rrt_fallback_20261119.json; CRA_PROJECT_CONTEXT.json.

### 2026-11-19 — Keep the workspace visible without camera data; YAML dataset defaults

User log diagnosis: the supplied launch excerpt configures rrt_shadow=True but
contains zero `RRT shadow` events, seven A* PATH_FOUND and zero A* NO_PATH.
There is no evidence of shadow-search admission in this excerpt. RRT remains
observation-only and never supplies Motion trajectories; HUMAN_RETREAT states
come from existing recovery. Enabling the observer does not force a search.
Evidence: test/results/camera_absent_log_diagnosis_20261119.json.

- Keep GNG nodes/edges at their normal green/orange colors and opacity for
  UNKNOWN, camera absence and expiry. The earlier gray/translucent style could
  make the graph appear missing; the log itself does not establish rendering
  loss. Missing human data now removes only human masks. Fresh target/obstacle
  boxes remain masked independently; fresh capsules still apply if simulation
  poses are stale. Text reports partial geometry without implying safety.
- XYZ mask output remains visual-only. Actual A* and CollisionFilter still
  check whole-arm configurations and joint-space transitions. No changes to
  RRT admission, active planning, Motion, margins or camera-loss/STOP policy.
- Add workspace.path and gng.graph_path to robot.yaml; main, Planning and
  Simulation launches read/expand these defaults. Missing fields preserve
  previous defaults; CLI overrides remain optional. Normal run is now simply
  `ros2 launch cra cra.launch.py rviz:=true`, without dataset arguments.
- Verification: 247/247 Planning tests pass in 29.642 s, including 25 mask
  tests. Five launch tests check YAML defaults, fallback, tilde expansion,
  overrides and the shipped dataset files. Native DDS domain 227 starts with
  no camera messages: 999/1000 nodes remain at alpha 0.85 (one hidden by target),
  and moving a simulation box without camera data changes visibility to 998.
  Human blocking/away, EMPTY, UNKNOWN and expiry pass. Zero Motion commands,
  additional mask workers or unchanged-scene marker/calculation repeats.
- planning/cra build succeeds in 1.41 s; installed --show-args confirms dataset
  defaults, and Flake8/CRLF-aware whitespace checks pass. This is headless
  transport verification; interactive RViz and moving-robot behavior were not
  tested. Existing running nodes must be restarted to load the correction.

Files: src/planning/planning/{planning_node.py,visualization/planning_visualizer.py,
visualization/workspace_graph_mask.py}; src/planning/test/{test_workspace_graph_mask.py,
verify_workspace_graph_mask_ros.py,WORKSPACE_GRAPH_MASK.md};
src/planning/test/results/camera_absent_mask_20261119.json;
src/cra/config/robot.yaml; src/cra/launch/{cra,planning,simulation}.launch.py;
src/cra/test/test_launch_data_paths.py; CRA_PROJECT_CONTEXT.json.

### 2026-11-19 — Replace heavy filtered display with lightweight workspace XYZ mask

Status: User-authorized replacement implemented, built and tested headlessly.
This supersedes the whole-arm filtered visualization described in older entries.

- Remove the progressive graph-filter worker, its extra CollisionFilter/model,
  filtered joint-node/edge publishers and `Joint graph - filtered` RViz display.
  Offline PathPlanner filtering methods remain available; the live display no
  longer calls them. No changes to A*, manager, CollisionFilter or Motion.
- `Workspace graph - obstacle mask` uses the exact stored GNG positions/edges
  (1000/4453), rather than overlaying the separate 4000-configuration joint
  graph. Hide XYZ nodes and straight edges intersecting validated human
  capsules or existing active simulation boxes, including the target. Restore
  original elements after obstacles move away; graph assets stay unchanged.
- Cache static endpoints and obstacle signatures; use bounding-box rejection
  and batched finite-segment distances. No FK, joint sampling, whole-arm
  collision queries, additional ROS node or process. Identical geometry does
  no mask calculation; unchanged visibility/status sends no MarkerArray.
  The native stable-scene probe observes zero additional calculations/messages.
- Mask display defaults on, original GNG defaults off to avoid covering hidden
  elements. New topics: /planning/workspace_graph_mask and its _status String.
  Persistent markers avoid refresh streaming. UNKNOWN/expired human or stale
  obstacle input restores a gray graph labelled unverified. A blocked main
  executor can retain the last snapshot until callbacks resume. XYZ occlusion
  never constitutes whole-arm clearance or authorizes Motion; full robot/path
  validation and existing execution/UNKNOWN experimental flags are preserved.
- Final regression: 244/244 Planning tests pass in 29.650 s, including 22 new
  mask tests replacing the 18 retired filtered-display tests. Coverage includes
  crossing edges with exterior endpoints, tangency, parallel/degenerate cases,
  rotated boxes, restoration, data identity, freshness and display isolation.
- Native DDS domain 227 uses the full GNG with real Planning subscriptions and
  synthetic obstacles. Human/box overlaps hide and restore elements; EMPTY,
  UNKNOWN and expiry behave as labelled. Zero Motion commands, zero mask
  processes and zero old filtered-topic publishers. Display calls are guarded
  against robot state/edge collision queries. RRT domain 224 marker regression
  also passes. Ordinary Planning/RRT timers are cancelled in the mask probe.
- Offline median XYZ-mask time: 0.54 ms for one capsule, 5.08 ms for 16 capsules,
  5.49 ms for 16 capsules plus a box, over 50 geometry changes on the stored GNG.
  These exclude ROS serialization, RViz rendering and existing planning work.
  planning/cra build succeeds (2.26 s); Flake8 and whitespace checks pass.

Files: planning/visualization/workspace_graph_mask.py; planning/planning_node.py;
planning/visualization/planning_visualizer.py; test/test_workspace_graph_mask.py;
test/verify_workspace_graph_mask_ros.py; test/benchmark_workspace_graph_mask.py;
test/WORKSPACE_GRAPH_MASK.md; test/results/workspace_graph_mask_20261119.json
(under src/planning), plus src/cra/config/cra.rviz and CRA_PROJECT_CONTEXT.json.
Retired display documentation redirects to the new guide; old result artifacts
are retained as historical evidence.
Next: restart the existing CRA/RViz launch and check human/box masking visually.
Interactive RViz, live Kinect noise and moving-robot avoidance remain unverified.

### 2026-11-19 — Bounded RRT tree markers, replay and empty-filter diagnostics

Status: Implemented, built and verified headlessly. Native interactive RViz,
live Kinect and moving-robot avoidance remain unverified.

- Reuse the existing shadow worker and CollisionFilter. Optional recording
  retains up to 256 accepted nodes per tree, 64 rejected candidates and 512
  events, with a reserved terminal event. Accepted children appear only after
  both state and directed edge checks pass; full path certification is retained.
  No GNG/A*, checker margin, Motion interface or experimental flag is changed.
- Share the existing MuJoCo FK/base_link projection between the visualizer and
  worker. Compute each distinct retained configuration once after search;
  cache/batch markers at <=5 Hz, with no live-callback FK/collision queries.
  Marker lifetime is <=0.6 s and capped by original snapshot expiry. Scene,
  target or freshness changes clear old trees. No automatic tree revalidation.
- Separate RViz Start Tree, Goal Tree, Final Path, Rejected Candidates and
  Search Step/Status displays. Rejected candidates default off. The green path
  is the certified baseline RRT snapshot, not a preference-selected/live path.
  RRT shadow still runs only after eligible primary graph failure. Trace/FK
  work requires marker subscribers and visualization_enabled=true.
- Existing filtered graph is retained. Its saved RViz checkbox defaults off,
  so the subscriber-triggered worker has no work until enabled. Freshness gates,
  node-first processing and geometry-driven progress resets can also leave the
  display empty/sparse. These are source-verified conditions; the user's exact
  live cause is unconfirmed. Expose /planning/filtered_graph_status counters
  and reasons; make the Displays dock visible. No duplicate filter is added.
- Offline recorder/replay uses the same search, checker and FK. Equal start/
  goal, seed 9: without human, direct path with 2 roots; with synthetic blocking
  capsule, direct edge rejected and a certified detour with 92 accepted nodes.
  It records 64 rejections and 512 events (88 omitted events explicitly
  reported). Human freshness is frozen only for this historical experiment.
  Replay labels historical data and reveals the final path at PATH_FOUND.
- Final Planning regression: 240/240 tests passed (18 new visualization tests),
  29.678 s. Actual geometry, invalid state/edge exclusion, directed parent
  certificates, bounded replay, expiry/target/revision gates, renderer isolation
  and FK versus five stored joint_eof configurations pass. planning/cra build,
  installed replay --help, Flake8 and whitespace checks pass. Initial combined
  runs exhausted MuJoCo 3.1.6 STL file descriptors; new tests reuse one model,
  and the final suite passes without raising OS limits.
- Native DDS domain 224 receives both tree roots and a certified path, then
  DELETE markers on revision change; domain 225 rechecks the existing filtered
  graph; domain 226 replays samples, nearest selections, extensions, connection
  steps and final path with historical labels. All probes emit zero Motion
  commands. Domain 224's two-node FK projection took ~0.105 ms; this is a small
  fixture measurement, not a full-tree performance guarantee. The ordinary
  Planning timer is cancelled in live-node transport probes.

Files: planning/algorithms/rrt_connect.py; planning/rrt_shadow.py;
planning/rrt_trace.py; planning/rrt_replay.py; planning/planning_node.py;
planning/visualization/{planning_visualizer,projection,rrt_display}.py; setup.py;
test/{test_rrt_visualization,record_rrt_experiment,verify_rrt_replay_ros,
verify_rrt_shadow_ros}.py; test/RRT_VISUALIZATION.md;
test/results/rrt_visualization_20261119.json (under src/planning), plus
src/cra/config/{cra.rviz,robot.yaml}. Pre-existing filtered-display work is kept.
Next: use the documented normal launch, enable filtered graph when needed,
inspect status while human geometry changes, and capture matching RViz views.
Keep historical replay separate from live ROS topics using its own domain.

### 2026-11-18 — Optional original/filtered RViz graph displays

User-requested visual-only feature after pushed checkpoint ba5b072. RViz has
independent `GNG - original` (on) and `Joint graph - filtered` (off) MarkerArray
checkboxes. The first preserves learned GNG points/edges; cyan filtered markers
show checked-valid joint configurations/edges projected at stored joint_eof.
These are 1000-point GNG versus 4000-configuration joint-graph layers.

- Separate spawned worker filters only while a filtered subscriber exists.
  One job, <=128 checks or cooperative 0.08 s per slice, independent model/data/
  cache. Parent does no new collision queries; original blocking full-filter
  call is removed from visualization. A frozen full-graph probe took ~55 s.
- Unknown/stale input clears markers. New geometry resets progress; identical
  geometry reuses checks while observations remain fresh. Unchecked points and
  connections are hidden, not labelled blocked. Text reports checked/valid and
  shown counts; 2000-edge drawing cap remains display-only. Marker lifetime
  is 0.6 s, and worker/render errors cannot change live Motion decisions.
- All 222 Planning functional tests pass, including 18 new display tests.
  Analytic geometry, optional subscription, progressive resets, expiry,
  saved RViz choices and marker/error-isolation tests pass. Native headless DDS
  probe on a temporary three-node joint graph shows 3 -> 2 -> 3 visible nodes
  as the synthetic human blocks/moves away; blocked edges 0/3, UNKNOWN deletes
  markers, zero Motion commands. Source hashes saved. Planning/cra build and
  Python style checks pass.
- Full dynamic graph/GUI interaction remains a live trial: human noise can
  repeatedly reset progress. A sparse/partial display does not prove NO_PATH.
  Existing A* is still synchronous. Graph assets, CollisionFilter, manager,
  Motion, 5 cm margin, 2 s timeout and experimental flags are unchanged.

Files: src/cra/config/cra.rviz; src/planning/planning/filtered_graph_display.py;
planning_node.py; visualization/planning_visualizer.py; test/test_filtered_graph_display.py;
test/verify_filtered_graph_ros.py; test/FILTERED_GRAPH_DISPLAY.md;
test/results/filtered_graph_ros_20261118.json (Planning paths under src/planning).
Next: relaunch normal CRA/RViz, tick filtered/untick original, and observe
progress and human-driven invalidation without treating partial masks as a plan.

### 2026-11-18 — Isolated RRT observation and latest-scene validation

Status: User-authorized observation stage implemented and built. GNG/A* still
supplies live paths. RRT candidates do not command Motion. No best-first
retreat or completed-task/idle-policy change is included.

- PlanningManager records failed graph attempts after its existing goal
  alternatives. A 50 ms node timer admits/polls one spawned-process job; no
  extra search is added to the live manager step. Worker owns model/data/cache.
  Inputs copy obstacle sizes/masks/poses and retain original human expiry.
- RRT and the clearance selector run observationally. A candidate is checked
  again using CRA's actual trajectory-path preparation and collision validation
  on a new latest scene. Target/attempt replacement, movement, stale feedback,
  human expiry and later scene changes prevent current-scene labeling. No live
  path assignment or STOP/RELEASE/EXECUTE comes from the observer.
- `planning.rrt_shadow.enabled=true` enables this only in simulation; disabling
  it restores the original node without observer jobs. Original 5 cm mandatory
  margin, 2 s human timeout and execution/UNKNOWN experimental flags remain.
  One submission per attempt, >=3 s spacing, no queued backlog. Running stale
  jobs drain under cooperative deadlines; worker errors disable only observer.
- All 204 Planning functional tests pass, including 22 new observer tests.
  Planning/cra build and new Python style checks pass. Actual headless DDS
  probe on isolated domain 224 receives 36 joint and 36 human updates while
  worker active, labels the latest-scene validation successful, and emits no
  Motion commands. Source hashes and event/timing evidence are saved.
- Probe scope: synthetic far human, artificial failed graph attempt and
  cancelled ordinary planning timer. Existing A*, recovery searches and final
  live path preparation remain synchronous; whole-pipeline responsiveness,
  physical camera behavior and robot execution are not established here.

Files: planning/rrt_shadow.py; planning/planning_node.py;
planning/planning_manager.py; test/test_rrt_shadow.py;
test/verify_rrt_shadow_ros.py; test/RRT_SHADOW.md;
test/results/rrt_shadow_ros_20261118.json (under src/planning), plus
src/cra/config/robot.yaml. Diagnostics: /planning/rrt_shadow and RRT shadow logs.
Next: collect normal simulation failure/freshness/revalidation timings before
choosing shared live budgets or active fallback dispatch.

### 2026-11-18 — Offline clearance preference and small human perturbations

Status: User-approved offline extension of the RRT prototype. No live caller,
launch configuration, motion command, best-first retreat or idle policy changed.

- Added `ClearancePathPreference`: score the baseline's minimum sampled
  clearance, try one RRT-Connect alternative around a preferred 7 cm band,
  select only if clearance improves and joint path length stays within 125%
  of baseline. Fixed start/goal clearances cap the preference. Mandatory 5 cm
  margin is unchanged. Baseline fallback requires fresh final certification;
  expiry, revision changes and final deadline failures return no path.
- Search uses isolated MuJoCo data/cache and the existing collision geometry.
  Ranking always requests all-pair mesh-box refinement at the original margin.
  Ordinary fast FAR reports may otherwise expose different-quality sphere
  lower bounds; those are not interchangeable with the refined ranking metric.
- Eight paired seeds in the blocked-direct capsule scene all returned baseline
  and preferred paths, independently prepared/certified. Median refined minimum
  clearance improved 5.1373 -> 7.0838 cm; median path length 4.8412 -> 5.0923 rad.
  Maximum individual extra travel was 20.46%, within the 25% cap.
- Unchanged paths were checked against +/-1, +/-3 and +/-5 mm capsule shifts
  on each axis. Baseline validity: 44/48, 34/48, 32/48; preferred: 48/48 at each
  amplitude. Both nominal scenes give 8/8. This is one synthetic static scene
  and deterministic perturbations, not measured sensor noise or moving humans.
- Selection overhead median 1.354 s (0.909-1.710 s), additional to baseline
  generation. Geometry freshness is frozen only for the experiment; real CPU
  deadlines advance. The 2 s selector budget is not a live feedback latency
  guarantee. Integrating synchronously into the current ROS pipeline is pending
  timing/freshness design, not authorized by these offline results.
- All 182 Planning functional tests pass, including 17 preference tests.
  Planning build, new Python style and installed imports pass. Benchmark source
  hashes match. No camera, ROS or robot execution trial ran.

Files: planning/algorithms/clearance_preference.py;
test/test_clearance_preference.py; test/benchmark_clearance_preference.py;
test/CLEARANCE_PREFERENCE.md; test/results/clearance_preference_20261118.json
(all under src/planning).
Next: discuss bounded fallback scheduling and fresh feedback before runtime
integration; preserve the existing GNG/A* and Motion safety protocol.

### 2026-11-18 — First bounded RRT-Connect prototype and offline comparison

Status: User-agreed first development experiment implemented, built and tested
offline. It is not called by PathPlanner/PlanningManager or the live launch.
Checkpoint 5e10ba9 remains the pre-experiment baseline. No runtime config,
Motion, GNG/A*, retreat guard or completed-task behavior changed.

- Added a standalone five-joint bidirectional RRT-Connect with ordinary CRA
  state/edge checks, full assembled-path certification, deterministic seeds,
  finite joint bounds and explicit iteration/node/cooperative wall budgets.
  It rejects stale human data, environment changes, invalid endpoints and
  late/checker-error results. It cannot escape an invalid human-margin start.
- 16 new tests use exact slab-intersection obstacles and exercise detours,
  reversed goal branches, invalid states, expiry, revisions, checker errors,
  final certification and bounded inner CONNECT expansion. All 165 Planning
  functional tests pass. Planning build, Python style and syntax checks pass.
- Offline benchmark: 8 seeds/repeats per method across five scenes, 80 trials
  total. Far-human and blocked-direct scenes both give 8/8 for A* and RRT.
  The deliberate two-node/no-edge graph plus blocked direct segment gives
  A* 0/8 and RRT 8/8 with real tree growth. This is an artificial graph-gap
  control, not a diagnosis of the live graph. Both reject the 3 cm non-contact
  start under the 5 cm margin and reject expired observations.
- All 40 returned paths are prepared under existing Motion segment limits
  and independently checked with new MuJoCo data, disabled geometry cache
  and twice-finer 0.01 rad sampling. Source/config/graph hashes are saved.
- Blocked-direct medians: existing A* 0.315 s/7.011 rad path; RRT 0.234 s/
  4.841 rad path. Human geometry is deliberately frozen; RRT's 0.5 s deadline
  uses real perf_counter. A* keeps its existing limits without a new wall
  deadline. These are not an equal-budget race or live timing guarantees.
- Some RRT paths graze the allowed margin: minimum sampled clearance
  0.050079 m. Frozen-scene success does not establish noise robustness,
  moving-human safety, continuous collision freedom or servo trackability.

Files: planning/algorithms/rrt_connect.py; test/test_rrt_connect.py;
test/benchmark_rrt_connect.py; test/RRT_CONNECT_PROTOTYPE.md;
test/results/rrt_connect_offline_20261118.json (all under src/planning).
Next: discuss clearance robustness and total feedback/freshness budgets before
live fallback dispatch; completed-task hold policy and best-first retreat
remain separate unimplemented work. No ROS/camera/robot trial was started.

### 2026-11-17 — Checkpoint before retreat-search algorithm experiments

The user requested saving and pushing the existing development before starting
RRT-Connect or best-first retreat. Both algorithms remain discussion proposals;
this checkpoint retains the current GNG/A* and local human-margin recovery.
It includes execution-progress validation, collision memoization, diagnostic
fixtures/replays, simulation profiling, Kinect timeout/cleanup changes, current
RViz presentation settings, console-only demo and the updated agent guidance.
Experimental settings remain human timeout 2 s, margin 0.05 m, cache 256,
execute_path=True and simulation_only_ignore_unknown=True.

Pre-checkpoint functional regression: 149 Planning tests and 14 Simulation
tests pass in cra-humble without camera/robot execution. General unittest
discovery also attempted the existing ament lint templates; those cannot load
because ament_copyright, ament_flake8 and ament_pep257 are not installed in the
container. Dedicated functional suites pass; the missing lint dependencies are
not represented as successful checks. JSON/YAML and CRLF-aware diff checks pass.
Earlier driver and demo verification and live-test limits remain documented
below. The local crash dump `core` is excluded from the checkpoint.

### 2026-11-17 — Enable every RViz display and add console-only demo entry point

Status: User-requested presentation changes implemented and cra build passed.

- All ten saved displays in `src/cra/config/cra.rviz` now have Enabled/Value
  true, including collision bounds, skeleton and both GNG layers. Installed
  config was parsed and verified; rendering still requires topic messages.
- `ros2 run cra demo` includes the existing cra.launch.py with rviz=true by
  default. Launch file handlers are directed to /dev/null before LaunchService
  starts. A global process prefix execs ROS nodes with external file logging
  disabled; process output remains on the terminal. Ctrl+C and respawn retain
  their existing process semantics. Normal ros2 launch logging is unchanged.
- Native headless comparison uses an actual tf2_ros C++ node and a small rclpy
  node on isolated localhost domain 217: normal mode creates launch/C++/Python
  logs; demo mode creates no log file or log directory, retains console INFO/
  WARNING and C++ output, and exits cleanly. Installed ros2 run help, Python
  style and CRLF-aware diff checks pass. No camera/robot/full CRA demo started.
- Motion/planning/camera settings unchanged. No rosbag started or old logs
  deleted. Temporary parameter files and explicit application recordings are
  outside ROS log-file control. Avoid tee when no saved output is desired.

Implementation: src/cra/scripts/demo, CMakeLists.txt and config/cra.rviz.
Usage: src/cra/DEMO.md. Reproduction: src/cra/test/test_demo_console.py.
Next: source install/setup.bash and run ros2 run cra demo for the live display.

### 2026-11-17 — Recover transient Kinect capture timeouts and clean up workers

Status: User-authorized driver correction implemented, built and tested in
`cra-humble`; real-camera recovery and full CRA execution remain unverified.

- The supplied 08:19 run starts K4A, then fails polling about 19 seconds later.
  SDK TIMEOUT returns false while FAILED throws; the old driver treated every
  false capture as fatal. Later full-queue messages do not prove initial load
  or USB failure. Physical cause of delayed capture remains undetermined.
- Driver now tolerates four consecutive timeouts and stops on the fifth,
  controlled by startup parameter `capture_max_consecutive_timeouts=5`.
  A real capture resets the counter. Missing captures skip publication entirely;
  neither old frames nor fabricated fresh timestamps are published.
- Worker exceptions are contained. Atomic stop control and interrupting
  camera/IMU/tracker waits before joining address shutdown ordering; only
  joinable workers are joined. No-device startup now prevents tracker creation
  with unavailable calibration and cleans up partial startup.
- Build succeeds with Body Tracking enabled. Twelve C++ tests and four native
  startup cases pass, including against the installed node. Test-only SDK shim
  denies all device opens and is not installed. Python style and diff checks
  pass. This is hardware-free verification, not live capture recovery proof.
- Camera FPS/resolution, Planning 2 s timeout and 5 cm margin, Motion and
  execution flags remain unchanged. Persistent SDK/USB failures can still
  require respawn; no physical device reset or frame-rate experiment was run.

Files: `src/azure_kinect_ros_driver/{src,include,test}/`, CMakeLists.txt and
package.xml. Reproduction, expected logs and verification limits are in
`src/azure_kinect_ros_driver/test/STREAM_RECOVERY.md`; evidence is in its
`test/results/` directory. Next: relaunch CRA once and verify fresh-frame
recovery and completed shutdown before considering a separate 15 FPS trial.

### 2026-11-17 — Diagnose latest stuck run and apply requested 5 cm human margin

Status: Inspected the 07:49:20 Asia/Jakarta run at its original 0.10 m margin;
applied the user's requested 0.05 m margin for the next simulation trial. Timeout
remains 2.0 s, cache 256, static margin 0.005 m and execution flags unchanged.

- Latest launch confirms 2 s/cache active. Of 15 profiled main path searches,
  ten succeed and five fail after capture expiry; failed calls spend 1.619-
  1.730 s against the remaining approximately 1.619-1.729 s budget. This does
  not prove that a longer detour is geometrically absent.
- Recorded retreat-block profiles include two HUMAN_OVERLAP, fourteen
  NO_MONOTONIC_STEP and one SEARCH_BUDGET_EXHAUSTED. Numeric proxy clearances
  reach -0.044946 m. Some stuck frames remain only 0.022-0.044 m clear, so a
  0.05 m margin still requires retreat. Negative clearance means overlap of
  conservative robot/human approximations, not proof of physical contact.
- Five recorded rejected path samples have clearances 0.080306-0.099472 m.
  Their threshold comparison changes at 5 cm, but full paths must still be
  revalidated; a single now-clear sample does not certify the rest of a route.
- Three Motion STALLED faults occur at 07:51:13.675, 07:51:47.260 and
  07:51:54.340. The final snapshot remains MOTION_ERROR/CONTROL_ERROR/STALLED
  with fault=True. Lowering human margin cannot clear this Motion fault.
- Existing stall monitor uses net measured displacement over 2 s against
  integrated absolute requested motion. Added an offline native C++ probe,
  compiling the unchanged ExecutionMonitor: reversing synthetic travel totals
  0.24 rad with nearly zero net displacement and triggers STALLED; truly
  stationary and monotonic controls respectively return STALLED and NORMAL.
  This proves a possible false-positive class, not the cause of the three live
  faults. Failure log joint labels select maximum tracking error rather than
  the actual stall-triggering joint. No latest full-joint/command bag exists;
  the new probe documentation supplies a uniquely named recording command.
- Camera driver also fails to poll at 07:50:07.398, restarts, then later aborts
  and repeatedly cannot reopen K4A. This is separate evidence for perception
  invalidity/loss, not a verified USB/driver root cause or a margin issue.
- Simulation's 38 complete windows average real-time factor 0.68971 across
  190.050 wall seconds, range 0.61069-0.80903. Clock scheduling is unchanged.
- Verification: four real-model fixed-scene controls show 4.9 cm rejected,
  5.1 cm and 7.5 cm accepted, and -0.5 cm overlap still rejected with
  HUMAN_OVERLAP. Eleven mesh-bound and thirteen cache tests pass; cra/planning
  build and changed Python style pass. Native installed PlanningNode constructor
  on isolated localhost domain 223 confirms 0.05 m human margin, timeout 2 s,
  cache and retained static margin/flags without spin or motion commands.

Only robot.yaml's human margin and a startup margin log change runtime files;
Motion/retreat/camera algorithms are unchanged. New probe is offline test code.
Evidence: src/simulation/test/diagnostics/stuck_20261117_074920_margin_diagnosis.json;
src/planning/test/results/human_margin_5cm_20261117_boundary.json;
src/motion/test/diagnostics/stall_monitor_probe_20261117.json. See
src/motion/test/STALL_MONITOR_PROBE.md for limits and recording instructions.
Next: relaunch with human_safety_margin=0.050m, record all joint feedback/path/
velocity commands to distinguish real immobility from reversal-related stall
classification, and discuss any detector/retreat change before implementing it.

### 2026-11-17 — Apply approved 2 s human timeout and exact collision memoization

Status: User-approved configuration/optimization implemented, built and tested.
Live CRA execution with the combined change still requires a new user launch.

- robot.yaml now sets planning.human_avoidance.data_timeout=2.0. Native installed
  PlanningNode construction verifies 2.0 s at both HumanCapsuleMonitor and
  CollisionFilter, retaining capture/receipt deadline checks and invalid-input
  handling. No timestamp is refreshed by a cache hit.
- CollisionFilter adds an exact, bounded 256-report LRU for human geometry.
  Its raw FK pose/radius, frame, mesh-box shape/value, capsule, margin, mode and
  revision key uses no rounding. Hits skip only repeated preparation/distance
  work; callers still run FK, static contacts, freshness and existing path/edge
  checks. Reports/pair maps are copied, UNKNOWN is not stored, and freshness/
  revision are checked after hits/misses. Snapshot updates, including identical
  stationary observations, and obstacle/enablement changes clear the cache.
- PathPlanner reads planning.collision_checks.human_report_cache_size (256;
  zero disables). Planning profile JSON exposes capacity/entries and cumulative
  hit/miss counters; startup reports human_geometry_cache=256. Existing A*
  graph caches/algorithm, collision samples, retreat iteration bound, Motion,
  human/static margins and experimental execution flags remain unchanged.
- All 149 Planning tests pass, including 13 new cache tests for receipt/capture
  expiry, mid-query expiry, new/identical observations, moving humans, static
  contact, report mutation, all-pair mode, geometry changes, capacity and random
  full-geometry equivalence. Changed Python and benchmark pass Flake8.
- Seven shuffled cold repeats per variant/scene, with frozen snapshot time:
  detour median 102.063 -> 82.526 ms; eight-capsule detour 108.177 -> 85.909 ms;
  margin retreat 139.145 -> 125.309 ms. Path hashes/failures match exactly, and
  every returned path is recertified with the cache disabled. Human geometry
  calculations drop 517 -> 241 for the detours and 424 -> 319 for retreat.
  These are approximately 19-21% lower planning and 10% lower retreat time on
  synthetic stationary fixtures, not universal live performance estimates.
- Separate real-elapsed stationary timeout comparison: both 1.5 s-age trials
  succeed with 2 s and are rejected with 1 s; 2.1 s-age inputs remain rejected
  under both. This checks deadline behavior, not moving-human execution.
- Builds for cra/planning pass and final installed source hashes match. A native
  ROS constructor smoke test on isolated localhost domain 223 verifies cache,
  timeout, margins 0.10/0.005 m and execute_path/simulation_only_ignore_unknown
  True, without spinning or publishing motion commands. Its startup-only log
  is not a new live trial and must not be mistaken for one when checking logs.

Files: src/cra/config/robot.yaml; active Planning collision_filter.py,
path_planner.py and planning_node.py; new cache tests and reproducible benchmark.
Documentation: src/planning/test/COLLISION_MEMOIZATION.md. Artifacts:
test/results/collision_memoization_20261117.json and
test/results/human_timeout_2s_memoized_20261117.json include hashes/per-trial data.
Next: relaunch CRA, check startup timeout/cache settings, and compare logged
path_search time, expiry failures, STOP reasons and cache totals on the same
scene. Simulation timing correction and global retreat search remain separate.

### 2026-11-17 — Inspect live simulation profiles from the 07:07:13 run

Status: Read-only inspection completed; no runtime/source/config behavior change.
Evidence is the latest simulation/planning/Motion per-node logs from 07:07:13
Asia/Jakarta, with hashes in simulation/test/diagnostics/
live_loop_20261117_070713_diagnosis.json and 216 per-window stage CSV rows.

- Profiler is active. Its 27 complete windows cover 135.0469 wall seconds and
  95.416 physics seconds: weighted real-time factor 0.70654, window range
  0.64917-0.78478. This directly verifies slow physics in the live viewer run.
- Wall shares: physics 2.413%, viewer_sync 16.639%, feedback_and_markers 12.432%,
  ros_callbacks 13.466%, actual sleep 49.608%. Mean feedback plus viewer event
  cost is 4.1148 ms against a 2 ms single-step budget. Work exceeds the step
  budget on 10,163/47,708 iterations (21.30%). Actual sleep totals 66.9945 s
  versus 56.2751 s requested; scheduling oversleep is also visible. These stage
  numbers do not isolate CPU contention versus viewer/driver internal causes.
- Source still advances one fixed physics step per loop and uses relative
  sleep for that iteration. Expensive five-step publication/viewer iterations
  lose time without a later absolute-deadline catch-up. A bounded scheduling
  correction is a proposal only; it does not directly solve A* or retreat cost.
- Motion logs 13 accepted trajectories and 10 cancellations, with no logged
  TRACKING_ERROR or STALLED in this run. Final snapshot is COMPLETED with target
  error 0.047438 m. This is not proof that all prior Motion faults are resolved.
- Of 19 profiled main path_search calls, 10 succeed and nine fail. Every failed
  call crosses its human deadline. Example cycle 273 has 737.187 ms remaining,
  spends 740.311 ms and ends at -3.134 ms. These are freshness failures rather
  than proof that a longer geometric route is absent.
- Separately, nine retreat states report SEARCH_BUDGET_EXHAUSTED while human
  data remains fresh. PathPlanner.find_human_margin_retreat uses a 32-iteration
  greedy local search, not an exhaustive global detour search. Other recorded
  retreat block reasons include ROBOT_STILL_MOVING and NO_MONOTONIC_STEP.
  Do not conflate the local iteration bound with the 1 s capture deadline.
- Three path-failure diagnostics reject waypoints with human clearances
  0.057928, 0.099721 and 0.092575 m against the 0.1 m margin. They are not the
  earlier backwards-connector failure in these recorded events. The ten STOP
  actions comprise five HUMAN_TOO_CLOSE, two HUMAN_RETREAT_UNSAFE, two
  HUMAN_PATH_BLOCKED and one HUMAN_RETREAT_CLEAR cancellation. Vision still
  emits 171 invalid-input rejection warnings; no new STOP action in this run
  is labeled VISION_INVALID. An existing STOP can still be held without a new
  action, so this does not prove perception invalidity is harmless.

Next discussion: compare bounded absolute-deadline scheduling, retaining the
existing per-step callbacks/control/collision model, with decoupling/reducing
viewer work. Test timing correction separately from planner freshness and local
retreat discovery. No algorithm, speed, margin or timeout change is made here.

### 2026-11-17 — Add passive simulation clock and loop-stage measurements

Status: User-authorized diagnostic instrumentation implemented, built and tested.
Live CRA/viewer timing still requires a new launch; no scheduling correction or
Motion/Planning safety change is included.

- Simulation now logs bounded Simulation profile JSON every five wall seconds.
  It directly compares actual MjData.time advancement with monotonic wall time,
  reporting real_time_factor, physics start/end, step/feedback rates, loop/work
  mean/max, work-over-budget counts and requested versus actual sleep.
- Exclusive count/sum/max statistics separate ros_callbacks, viewer_lock_wait,
  control, physics, feedback_and_markers, viewer_sync, sleep and profile_output.
  Output serialization/logging is attributed to the next observation window;
  unattributed_wall_ms retains bookkeeping/report-construction and other gaps.
  Parent loop/work times are inclusive and must not be added to leaf totals.
- ROS construction parameters profile_simulation=True and
  simulation_profile_interval=5.0 control initial configuration. They do not
  dynamically reconfigure the accumulator. Startup reports enabled/interval,
  fixed physics timestep and feedback stride; no new ROS topic is introduced.
- Preserved: one existing physics step per iteration, five-step feedback/viewer
  cadence, callback/control/physics order, original sleep calculation, actuator
  control, ROS interfaces and safety parameters. Measurement/logging has observer
  overhead; no claim of zero performance impact is made.
- Six existing hold/tracking tests and eight new profiling tests pass. The
  controlled integration tests use the production run loop/control and actual
  MuJoCo with fake viewer/ROS spin/wall clock. Enabled/disabled profiling gives
  identical physical state, sleep requests and operation order; injected viewer
  delay is correctly attributed and lowers measured real-time factor.
- Changed Python passes Flake8 at 99 columns. colcon build --packages-select
  simulation passes. Installed source hashes match. A native ROS constructor
  smoke test in isolated localhost domain 223 verifies imports, parameters and
  model timestep without running a viewer/loop or publishing motion commands.

Files: simulation/loop_profiling.py, simulation/robot_simulation.py,
test/test_loop_profiling.py and test/SIMULATION_PROFILING.md in src/simulation.
Controlled evidence: test/results/loop_profiling_20261117_verification.json.
These synthetic timings do not establish the source of slowdown in the live
viewer. Restart the prior launch, record idle/moving/near-human phases and inspect
new Simulation profile lines before deciding on scheduling or viewer changes.

### 2026-11-17 — Reproduce joint-5 tracking fault and isolate simulation timing

Status: Authorized offline diagnosis completed. No runtime controller, guard,
planner, model, timeout or simulation scheduling behavior was changed.

- Added motion/test/replay_recorded_tracking.py, an offline C++ bridge and a
  minimal recorded trajectory/feedback/scene fixture. The bridge compiles the
  current production JointController, TrajectoryGenerator, MotionGuard and
  ExecutionMonitor. Physics tests call real RobotSimulation control methods
  with real MuJoCo; only ROS imports are stubbed, no nodes/publishers are started.
- Native Motion with recorded feedback reproduces joint-5 TRACKING_ERROR at
  2.245 s, versus approximately 2.248 s in the live acceptance/failure logs.
  Replay error is 0.21083 rad and guarded speed 0.5 rad/s. This reproduces the
  monitor decision, not the entire original ROS/physics execution.
- Same 11-point trajectory and unchanged configuration completes in headless
  real-time physics at 13.860 s, peak error 0.03067 rad, zero contacts. Making
  physics advance at half the controller wall-time rate instead produces a
  joint-5 TRACKING_ERROR at 3.245 s. An interpolated recorded-cadence physics
  schedule also faults joint5 at 2.585 s; exact timing is not a reconstruction
  because initial actuator state and callback schedules were not recorded.
- Recorded executing feedback spans 2.222912 wall seconds over 106 intervals.
  Source publishes every five 0.002 s steps: inferred physics span 1.06 s,
  physics/wall ratio 0.47685. Independent angle-change/reported-velocity
  estimates give a median 0.010002 physics seconds per feedback interval,
  consistent with five steps and supporting the slow-physics interpretation.
  Direct physics-clock telemetry was not recorded; counts alone would depend
  on no message loss, so the independent velocity check matters.
- Theory/code link: Motion uses steady_clock elapsed time and a 5 ms wall timer;
  RobotSimulation integrates velocity with fixed physics dt and advances one
  step per loop iteration. At ratio 0.477, the 0.5 rad/physics-second cap yields
  only about 0.2385 rad/wall-second for a 0.3 rad/wall-second reference. Slow
  simulation is sufficient to cause this fault with unchanged control guards.
  This does not establish the cause of all faults or which live loop stage
  consumes time (viewer, publication, callbacks, scheduling or physics).
- Verified: seven replay comparison checks pass; six existing simulation
  hold/tracking tests pass; diagnostic Python passes Flake8 at 99 columns.
  Installed simulation source, YAML and XML match those used by the replay.
  No end-to-end ROS/viewer run was performed.

Artifacts: motion/test/results/tracking_20261117_005856/diagnosis.json and four
CSV traces; motion/test/TRACKING_REPLAY.md documents reproduction and limits.
Next: measure physics/wall ratio and loop-stage timings in the live simulation
before selecting a scheduling/viewer correction. Retain all safety checks;
larger tracking thresholds, higher gain or human timeout do not solve this clock
rate mismatch. Vision validity and collision-search expiry remain separate work.

### 2026-11-17 — Diagnose remaining near-human delays after segment tracking

Status: Read-only inspection of the 00:57:02 launch, Planning/Motion profiles and
the finalized nine-topic bag cra_path_diag_1s_20261117_005856. No controller,
planner, perception or timeout changes are made by this diagnostic task.

- Startup confirms segment_progress=True, progress_corridor=0.04rad and a 1 s
  human timeout. Four emitted profiles show SEGMENT_PROGRESS_VALIDATED. The five
  failed-path events reject WAYPOINT checks (three remaining-execution and
  two planned-path events); none
  is the previous failed measured-to-old-waypoint connector. This does not prove
  all possible progress ambiguities have been solved.
- The wider log covers 304.403 s and 297 sampled profile rows. Of 54 main
  path_search stages, 39 return FALSE and all 39 cross the human deadline.
  Median stage time is 560.740 ms, maximum 703.780 ms. The launch separately
  contains 40 PathPlanner NO_PATH expiry messages: 38 graph-search and two
  final-validation expiries. These message counts are not assumed one-to-one
  with the profiled main stage. No geometric NO_GRAPH_PATH conclusion follows.
- Bag coverage is only 20.173189 s, 3,011 messages across all nine topics. It
  contains 77 valid observed and seven invalid human messages. Observed capture
  age at recorder receipt: median 0.272817 s, p95 0.327376 s, max 0.369748 s.
  In bag-window cycle 555, capture age is 397.546 ms at planning start and
  path_search has 581.610 ms left; it spends 582.441 ms and ends at -0.844 ms.
  Four profiled searches expire and one succeeds within this bag window.
- Fourteen of 20 manager STOP actions in the wider log follow VISION_INVALID;
  three follow fresh-data PATH_COLLISION, two HUMAN_TOO_CLOSE and one cancellation
  after HUMAN_RETREAT_CLEAR. An upstream valid=False message is different from
  capture-age expiry. Reconstruction checks input recency, synchronization,
  dimensions/frame/body-index validity, but the message carries no detailed
  invalidity reason, so the responsible upstream predicate remains unproven.
- Retreat also has timeout blocks, ROBOT_STILL_MOVING, one HUMAN_OVERLAP case
  and failed revalidation. Close/overlapping starts are not equivalent to an
  ordinary valid-start A* query. These guards are not bypassed.
- Motion reports five TRACKING_ERROR faults and one STALLED fault in the wider
  log. In the bag it starts executing about 5.942 s after recording begins,
  then enters CONTROL_ERROR/TRACKING_ERROR at 8.191 s; later profiles show
  recovery/no-path and MOTION_ERROR. Thus a motion fault also explains stopped
  behavior during this experiment, independently of finding a geometric path.
- The recorded joint-5 tracking fault has 0.212 rad error, command 0.724 rad/s
  and filtered command 0.500 rad/s. Velocity saturation is directly observed.
  Complete controller/plant cause still requires headless reproduction and
  reference/applied-command/force telemetry; do not assert saturation alone
  establishes a unique cause. Current source already reserves trajectory speed
  at 0.6 * joint.speed (0.3 rad/s) while MotionGuard caps at joint.speed (0.5).
  Adding that existing reserve is not a new correction. The STALLED log's joint
  index is chosen by maximum tracking error, not necessarily the stall-triggering
  joint, so its joint label is not sufficient mechanical-stall localization.

Theory and recommended next discussion: planner geometry/freshness, upstream
perception validity and closed-loop trajectory tracking are separate conditions
for motion. A longer timeout addresses only some expiry failures. Reproduce the
tracking fault with the recorded trajectory/controller/MuJoCo in isolation,
identify Vision invalidity reasons, and reduce repeated same-snapshot collision
work without weakening freshness or margins. A stationary-human 2 s comparison
can isolate deadline sensitivity but does not resolve invalid inputs or faults.
Changing a safety-sensitive controller or planner still requires discussion.

Artifacts: test/results/close_human_20261117_005702_diagnosis.json stores hashes,
bag statistics, timing/fault evidence, failed paths, hypotheses and limits. Its
adjacent stages CSV stores sampled stage measurements. Checks verify 39 profiled
expiry crossings, 40 launch expiry messages, 14 invalid-input STOPs, six Motion
faults and 3,011 recorded messages. Five-minute and 20-second statistics must not
be presented as if they share the same recording interval.

### 2026-11-17 — Implement ordered segment progress with collision-gated acceptance

Status: User approved development of the previously discussed Planning-side
tracker. The correction targets the recorded missed-intermediate-waypoint case;
GNG/A*, Motion's controller/final tolerance, ROS interfaces and collision rules
are preserved. Human timeout remains 1.0 s and human/static margins 0.10/0.005 m.

Theory and implementation:

- planning/execution_progress.py contains OrderedSegmentProgress. It projects
  measured joints onto only the active incoming segment and its adjacent outgoing
  segment. Projection is Euclidean; corridor error uses maximum joint deviation.
  It never searches globally for a nearest future waypoint or segment.
- The incoming segment must first be approached forward within its corridor
  beyond handoff_fraction. A missed-corner handoff is proposed only after two
  consecutive moving samples fit the outgoing segment, move forward on it and
  lie outside the old segment corridor. Endpoints and three interior feedback
  chord probes must fit the local corridor union. This is sampled evidence,
  not a continuous-motion safety proof. Stationary/backward samples cannot
  accumulate outgoing confirmation; clear backward retreat disarms the approach.
- Existing direct waypoint/chord passage remains unchanged and can still
  recognize directly crossed waypoints. The new estimator supplements it for
  slightly missed corners; final arrival remains Motion's responsibility.
- Feedback gaps/jumps, invalid local evidence and leaving the corridor discard
  unvalidated segment candidates. Ambiguous overlapping/reversing segments are
  deliberately conservative and may still require direct passage or replanning.
- PlanningManager keeps candidate and accepted cursors distinct. Joint callbacks
  perform only local arithmetic. The existing human guard validates the candidate
  forward remainder in the Planning cycle, including an unchanged environment
  revision. It accepts the cursor only after current state, remaining path,
  human freshness, scene revision, joint-feedback freshness and obstacle freshness
  pass. Failure still follows STOP/retained-path/replan handling as appropriate.
- Diagnostic index mapping uses the checked candidate cursor while also showing
  the accepted cursor. Profiles include next_waypoint, candidate_next_waypoint,
  progress_reason and segment_evidence_reason. Startup prints segment_progress
  and progress_corridor. Resetting/replacing a command resets segment evidence.

Configuration in planning.execution_progress in robot.yaml:

- enabled: true (false restores the original direct-passage behavior).
- corridor_tolerance: 0.04 rad, separate from Motion joint goal tolerance 0.02.
- confirmation_samples: 2; handoff_fraction: 0.9; max_feedback_step: 0.05 rad
  for new segment evidence. Existing direct-passage behavior retains its own
  prior feedback rules. No collision sampling step or safety margin is enlarged.
- Recorded estimator trials at 0.02/0.03/0.04 rad reached next cursors 5/6/13;
  0.04 rad is an experiment setting based on this recording, not a universally
  established tracking corridor. Live feedback cadence and noise still matter.

Verification:

- 136 unittest cases pass in cra-humble with the ROS environment sourced.
  New cases cover adjacent handoffs, two-sample confirmation, stationary/backward
  noise, future-loop proximity, overlap ambiguity, corridor/jump rejection,
  same-revision validation, blocked candidates, human expiry/revision/feedback
  expiry during validation, disabled-mode restoration and recorded missed corners.
  Thirteen focused tracker/profiling tests also pass after final diagnostic edits.
- A joint-only fixture retains the 1,226 recorded command-5 feedback samples and
  actual 14-point path. Estimator regression reproduces old cursor 5 versus new
  cursor 13; this test alone is not collision authorization.
- Full manager/checker offline control uses recorded feedback with FAR synthetic
  human geometry during traversal, then installs the exact recorded failure frame
  at the final measured pose. Old behavior returns STOP/HUMAN_PATH_BLOCKED at
  cursor 5; new behavior keeps the original command and returns NONE/EXECUTING
  at cursor 13. Collision guards run at about 5 Hz. Time is frozen for isolation;
  this is not a moving-human replay, live scheduling test or published command.
- test/results/segment_progress_replay_2026-11-17.json stores configuration,
  source hashes, cursor history, decisions and limits. The regression fixture is
  test/fixtures/recorded_progress_235941.json. Changed Python files pass Flake8
  with 99-column limit. Live MuJoCo/ROS execution of this correction remains
  unverified until restart and a new repeatable log/bag experiment.
- colcon build --packages-select cra planning completes successfully in
  cra-humble. Installed tracker/manager/planner/node copies match source and
  installed YAML retains the selected progress settings, 1 s timeout, original
  human/static margins and 0.02 rad final joint tolerance. JSON/whitespace checks
  pass. Running nodes must be restarted to load these changes.

### 2026-11-17 — Persist the user's teaching preference

The user requested lecturer-style explanations automatically in every CRA
conversation: reasons, problem/evidence, fundamentals, algorithm/theory, actual
implementation and behavior, concrete examples, then choices and verification.
Section 15 of AGENTS.md records this preference, including concise Indonesian,
intuition before equations and a different explanation when the user is unclear.
Depth scales to the question; simple updates do not require a long lecture.
CRA_PROJECT_CONTEXT.json mirrors the learning preference. The existing file
named 'AGENTS.md ' had a trailing space; it is renamed to the standard AGENTS.md
while preserving its prior contents. Robot code and experimental settings are
unchanged by this instruction update.

### 2026-11-17 — Prove the stale progress cursor from the nine-topic recording

Status: Read-only inspection of cra_path_diag_1s_20261116_235941, the matching
23:58:47 launch and Planning/Motion logs, plus offline original-algorithm replay.
No runtime code/configuration changes, rosbag play or robot commands were made.

- The finalized bag spans 27.780070 s with 6,249 messages and all nine requested
  topics. It contains 2,004 measured joint samples, 193 valid observed human
  frames, two actual joint-path commands and ten previews. Discovery delays
  human/Motion status coverage; earlier failures 1–2 precede the bag.
- The wider live log has 140 sampled profiles and six path-failure events.
  Events 1–2 reject parts ahead of measured progress according to their traces:
  command 3 edge 5–6 while nearest segment is 1, then command 4 waypoint 4 while
  nearest segment is 0. Events 3–5 reject planned paths while already stopped.
  These differ from the stale-cursor failure below; do not label all STOPs false.
- Event 6, cycle 375, occurs at 2026-11-17 00:00:07.590 Asia/Jakarta, about
  16.946 s after command 5 (14 points) is recorded. Planning's ordered cursor
  remains 5, while nearest segment is 12 at fraction 0.347970 with L2 error
  0.013460 rad. Its checked first edge goes from measured current_q backward
  toward command waypoint 5, max-joint displacement 1.871823 rad. No backward
  command is sent: the checker rejects that hypothetical connector and STOPs.
- Replay the actual _advance_execution_progress method over all 1,226 recorded
  joint samples during command 5. It advances to 2,3,4,5 at approximately
  0.800,2.688,4.402,5.301 s and never passes waypoint 5. Minimum max-joint error
  to that waypoint across measured samples and the implementation's projected
  feedback chords is 0.020875265 rad, just outside the 0.02 rad passage rule.
  Later states approach waypoints 9,10,11,12 within 0.02 rad in that temporal
  order, at about 12.018,13.645,15.450,16.556 s. Thus this is more evidence than
  merely choosing a globally nearest future segment.
- Motion's JointController samples the trajectory by elapsed time; the trajectory
  generator computes segment durations from joint changes and velocity/curve
  settings. It does not wait for measured arrival at every intermediate point.
  Planning reuses the final joint goal tolerance as its passage threshold, so a
  slightly missed intermediate point can leave its cursor behind Motion.
  Recorded time_from_start values are zero; use Motion acceptance/recorded times
  rather than interpreting those message fields as a trajectory schedule.
- Match the exact source human frame from cycle 375 (capture stamp
  1794848407327306578). Reconstruct actual recorded geometry using logged q.
  Current clearance reproduces 0.167573894 m exactly. The original remaining
  path current_q + command[5:] reproduces the failed first-edge sample 20/94
  and clearance 0.099852602 m on link5_visual versus body 1, segment 6.
- On the SAME frozen geometry, the forward tail current_q + command[13:] passes
  unchanged static and human state/edge checks, with minimum sampled human
  clearance 0.132378376 m (12 samples). Freezing lifetime isolates geometry for
  diagnosis only; this tail is never authorized or sent as a live command.
  Offline model revision numbers differ from live ones and are not frame IDs.
- These checks confirm a stale-cursor cause for event 6. They do not authorize
  nearest-waypoint jumping, establish a universally safe progress tolerance or
  explain away the earlier ahead-of-progress/planned-path rejections. Motion
  ultimately completes a new two-point trajectory after STOP/replanning.

Evidence: test/results/path_progress_20261116_235941_diagnosis.json contains bag
hashes, six events, the actual command, original-method replay, chronology and
same-frame geometric checks. The adjacent waypoint_passage.csv records each
waypoint's minimum measured/chord error. Consistency checks verify topic totals,
exact command/cursor correspondence, clearance reproduction and forward-tail
validity. The first broad container-inspection approval review timed out; one
read-only retry succeeded and no access remained blocked.

Next discussion: distinguish intermediate segment progress from final-position
settling. Compare ordered segment-passage evidence with an authoritative Motion
segment acknowledgement; a larger waypoint tolerance alone is a limited option.
Any selected approach must preserve ordered traversal on loops, feedback validity
and fresh collision validation of the forward remainder. Keep the 1 s timeout
and 0.10 m human margin while choosing the progress correction.

### 2026-11-16 — Diagnose rejected path locations and ordered progress

Status: User authorized failed-segment/progress diagnostics as the next step.
No cache optimization, waypoint-progress rule change, new planner, collision
margin change or timeout increase is included. Timeout remains 1.0 s and the
existing GNG/A*, execute_path=True and simulation_only_ignore_unknown=True remain.

- CollisionFilter retains the rejection already computed by is_state_valid:
  invalid joints, unknown human data, the first rejected MuJoCo contact pair,
  or the existing human clearance report (robot geom, body/segment, radii and
  geometry model). No diagnostic re-query of MuJoCo or capsules is performed.
- Existing path traversal is preserved: all waypoints first, then sampled
  edges. CollisionFilter and PathPlanner identify the first REJECTED CHECK
  in that order, not necessarily the earliest obstruction along arc length.
  WAYPOINT records contain point_index/point_q. EDGE_SAMPLE records contain
  edge_index, endpoints, sample_index/sample_count, fraction and failed q.
  Indices are zero-based; sampling resolution and boolean results are unchanged.
- PlanningManager snapshots evidence before invalidating the route, including
  stage, path source, environment revision, current measured-state report and
  remaining human lifetime. Guard failures, preparation failures with an actual
  recorded trace, and normal final-validation failures are covered. Missing
  traces are marked UNAVAILABLE rather than re-running geometry or inventing
  a cause. Dedicated retreat certificate internals are not instrumented here.
- For a measured remaining route, local point zero is measured current_q;
  later local point j maps to commanded waypoint next_waypoint+j-1. The first
  edge is explicitly marked as the measured-to-ordered-waypoint connector.
  Prepared-path indices refer to the prepared path; they are not mapped to a
  previous command because subdivision may change the waypoint count.
- Each EXECUTE decision receives a diagnostic commanded_path_id (a local
  attempt counter; publishing/acceptance must still be checked in the bag).
  Failure events include
  the ordered cursor, next-waypoint joint error and the last passage-update
  reason. Read-only nearest waypoint/segment projections provide comparison
  evidence and explicitly do NOT authorize progress. Future loops or feedback
  that misses a corner must not move the ordered cursor based on proximity.
- PlanningNode emits one structured 'Path validation failure: ' JSON line per
  event after Motion command dispatch, retaining the event after invalidation.
  Reports are bounded to the most recent failure and do not depend on profiling
  being enabled; a profiling cycle ID is supplied when profiling is active.
  Profiling lines include path_failure_id and commanded_path_id for correlation.

Review example: test/results/path_failure_diagnostics_offline_example_2026-11-16.json
comes from the existing synthetic DETOUR fixture with actual MuJoCo geometry
and a frozen clock. Current pose is FAR, commanded waypoint 1 fails on
link3_visual against capsule (body=0, segment=0), and the manager returns
STOP/HUMAN_PATH_BLOCKED. This is a diagnostic fixture, not the earlier live
human scene or live Motion execution. It uses the approved 1 s checker budget.

Next live test: save launch output as before, confirm the 1 s/profiling startup
values, keep target/human conditions repeatable and record the previous seven
topics PLUS /motion/joint_path and /planning/joint_path_preview. These existing
JointTrajectory topics allow diagnostic local indices and commanded_path_id
order to be reconciled with actual commands. Live failure localization remains
unverified until a new log/bag is captured.

Verification: 123 unittest cases pass in the ROS-sourced cra-humble environment,
including six new dynamic cases and updated inert-publisher node-order checks.
Tests cover actual human/contact rejection details, interior edge sampling with
the original query sequence, snapshot survival after invalidation, expiry versus
geometry, finite JSON, and a future-nearest waypoint that cannot advance progress.
The first unsourced discovery attempt failed to import three ament lint modules;
sourcing /opt/ros/humble/setup.bash resolves those imports and the full unittest
discovery passes. Those pytest-style ament lint functions are not executed by
unittest; the changed Python files are checked separately with Flake8 (99 columns).
JSON/YAML and git diff whitespace checks pass. No live robot or ROS transport test
is claimed by these offline checks. The Planning package builds successfully;
the new path_diagnostics module and failure-recorder method import through the
installed overlay without creating a ROS node.

### 2026-11-16 — Inspect live profiling from 19:37:16

Status: Read-only analysis of cra_profile_1s_20261116_193716.log in container
/tmp plus matching Planning/Motion logs. Startup confirms 1.000 s and profiling
enabled. No new rosbag metadata is found in container /tmp; no algorithms,
timeouts, controller settings or collision rules are modified by this analysis.

- Parse 106 JSON profile lines covering sampled cycles 1–458 over 93.200 s.
  Fast unchanged cycles are sampled, so these are not all-cycle distributions.
  Motion receives nine trajectories and eight STOP decisions. Seven STOPs
  report PATH_COLLISION with 606–725 ms human lifetime remaining; the eighth
  reports VISION_INVALID. A further PATH_COLLISION decision occurs while
  already stopped. No retreat stages/HUMAN_TOO_CLOSE occur in this run.
- Twelve recorded path_search stages with observed humans include eight
  successes and four expiry aborts. Median duration is 277.076 ms, maximum
  719.538 ms. All four PathPlanner NO_PATH launch messages explicitly report
  expired human data during graph search. Cycle 230 starts path_search with
  718.456 ms remaining, spends 719.538 ms and ends at -1.101 ms. This measures
  the combined PathPlanner/A* stage, not A* queue operations in isolation.
- Other observed-human stage medians/maxima: remaining-path validation
  102.630/211.275 ms (55 samples), execution preparation 77.959/114.048 ms
  (14), final validation 76.437/196.364 ms (14), observed RELEASE response
  8.004/91.975 ms (6), visualization 0.017/7.803 ms (87).
- Cycle 276 retains the same revision 80 while validating the remaining path
  (81.676 ms), preparing/validating the execution path (88.179 ms), then
  validating it again (196.364 ms). Manager total is 368.055 ms. The first
  three costs are sequential; human_guard and manager_step are inclusive
  totals and must not be added again. This supports investigating repeated
  collision work before changing the timeout, executor or planning algorithm.
- A* already caches graph states/edges per environment revision. The raw
  CollisionFilter path checker repeatedly evaluates states and sampled edges;
  preparation may subdivide the path, so its input is not necessarily exactly
  the same as the retained preview. Any reuse must establish input identity,
  current environment/settings and unexpired data rather than removing checks.
- PATH_COLLISION here proves rejection of the stored remainder while fresh.
  Without the raw capsules, commanded waypoints and continuous measured joints,
  it does not identify the failing segment or establish whether conservative
  waypoint progress retained an already passed segment. Next diagnostics should
  locate the failed segment and compare it with measured execution progress.
- Planning reaches COMPLETED at 1794832720.913797 s. Final snapshot target error
  is 0.098655 m inside the configured 0.12 m tolerance. Human observations vary
  OBSERVED/EMPTY/INVALID; no continuous close-human completion claim follows.
- Separate launch error: visual_node exits with code -5. Planning continues
  receiving human geometry; the crash is not established as the cause of the
  four timed search aborts. Its GUI/display cause remains uninvestigated.

Artifacts: test/results/planning_profile_20261116_193716_diagnosis.json contains
source log hashes, parsed profiles, stage summaries, deadline crossings, STOP
decisions and limitations; planning_profile_20261116_193716_stages.csv exposes
individual timing rows. Consistency checks verify unique cycles, nonnegative
durations, four search/launch expiry matches and seven fresh-data path STOPs.
Recommendation for discussion: keep 1 s while investigating repeated collision
work and failed-path location/progress. A 2 s comparison may admit longer search
but does not address the seven fresh-data rejections or invalid Vision input.

### 2026-11-16 — Instrument the 1 second planning experiment

Status: User authorized timing diagnostics before choosing another timeout or
algorithm. PlanningNode's startup-only ROS parameter profile_planning defaults
to True for this experiment and appears in its initialization log. Library
PlanningManager instances default to profiling disabled. Human timeout remains
1.0 s, margins remain 0.10/0.005 m, and execute_path=True plus
simulation_only_ignore_unknown=True remain unchanged.

Implementation:

- planning/profiling.py records bounded per-cycle measurements with
  time.perf_counter, preserves returned objects/booleans and propagates
  original exceptions. Remaining human lifetime is read from the existing
  minimum of receipt timeout and absolute capture deadline using monotonic
  time. Profiling never refreshes timestamps or authorizes a motion decision.
- Manager stages distinguish goal_selection, path_search,
  alternative_goal_search, current_clearance, remaining_path_validation,
  retreat_search, retreat_candidate_validation, retreat_guard_validation,
  execution_prepare_path/retreat and execution_final_path/retreat. Inclusive
  human_guard, retreat_guard, retreat_preparation and manager_step timings
  provide totals. Nested stages must not be summed as disjoint work.
- STOP/RELEASE/EXECUTE acknowledgement intervals are queued for the next cycle
  with origin_cycle. These measure time from the manager's command decision
  to its observed Motion response, including dispatch/callback scheduling,
  not pure DDS transport latency. Environment revisions are recorded before
  and after stages/acknowledgements to identify changing observations.
- Node records source capture timestamp, capture_to_callback_ms and
  capture_age_start_ms. The first age includes camera/transport/executor delay;
  their difference measures subsequent residence before the planning cycle,
  rather than isolating one source of latency. Unknown data has no certified
  remaining budget (null); an expired observed deadline has a negative budget.
- Trajectory building and visualization are timed separately. Structured JSON
  lines prefixed 'Planning profile: ' are emitted after Motion dispatch and
  visualization, including action/status and validation reasons. Slow cycles
  (>=100 ms), status changes, commands, exceptions and acknowledgements are
  logged. Identical fast idle cycles are sampled every 5 s. cycle_work_ms
  excludes writing the profiling line itself; profiling has measurement/log
  overhead and is not a promise of identical wall-clock timing.

Controlled next test: restart the built launch, confirm profile_planning=True
and human_data_timeout=1.000s, keep the human/target stationary, and save both
the launch output and the same seven rosbag topics for roughly 30 s. A bag of
status topics alone does not contain the new profiling JSON. Use a new
cra_profile_1s timestamped name and preserve existing baseline recordings.
Live timing and Motion execution with this instrumented build remain untested.

Verification: all 87 dynamic-replanning regressions pass with default library
profiling disabled and again with profiling enabled across the suite. Six
dedicated tests pass for absolute expiry/passivity, false/exception preservation,
bounded storage, acknowledgement cancellation/accounting and actual node-method
dispatch-before-log ordering using inert publishers. Flake8 (99 columns), JSON
parsing, installed timeout/margin assertions and git diff whitespace checks pass.
The Planning package builds successfully in cra-humble. Node-method tests are
headless and do not verify ROS transport, GUI behavior or live robot execution.

### 2026-11-16 — Inspect the recorded 1 second live trial

Status: Read-only inspection of /tmp/cra_stuck_1s_20261116_174406 in
cra-humble and the matching Planning/Motion logs. Startup confirms
human_data_timeout=1.000s, execute_path=True and
simulation_only_ignore_unknown=True. No runtime settings or algorithms changed.

- Bag duration 24.403726 s, 4,617 messages across all seven requested topics.
  Human geometry contains 201 valid OBSERVED messages, all base_link with
  eight capsules; no EMPTY or INVALID messages within this bag. Receipt age
  median is 0.254337 s, p95 0.276216 s, maximum 0.286261 s. The largest
  interval between recorded human messages is 0.429982 s.
- Subscription discovery delays coverage: human geometry begins 2.571447 s
  after bag start and joints/markers begin later. Motion's first recorded
  state is STOPPED at 2.535708 s; do not assign that state to the preceding
  unobserved interval when computing dwell times.
- Planning reports HUMAN_RETREAT_READY ten times and HUMAN_RETREAT_SENT
  twice. Motion accepts both retreat trajectories; the first reaches
  COMPLETED, while the second is cancelled after HUMAN_RETREAT_CLEAR.
  Motion also accepts four ordinary trajectories during the recording.
  First measured joint change greater than 0.01 rad is at 5.486713 s.
  The sole recorded Motion failure value is NONE; no tracking fault occurs.
- Instability remains: topic transitions include HUMAN_RETREAT_DATA_UNKNOWN
  six times, HUMAN_RETREAT_UNSAFE twice, HUMAN_VISION_UNKNOWN twice,
  HUMAN_TOO_CLOSE twice and HUMAN_PATH_BLOCKED three times. These are
  status transitions, not counts of every search or every failed check.
- Eight capsule/clearance matches use the raw recorded geometry and joint
  pose printed in live diagnostics. One frame arrives at age 0.259429 s but
  its retreat-ready diagnostic is emitted at age 0.970399 s. This leaves
  about 30 ms of the 1 s lifetime before subsequent validation. The elapsed
  0.710970 s includes callback waiting and decision computation; it cannot
  be attributed wholly to either without profiling. Planning uses rclpy.spin
  with synchronous searches/checks in planning_cycle, so expensive work can
  delay consumption of newer frames. No source-frame gap above 1 s is seen
  within the recorded human-topic coverage.
- Frozen-lifetime offline reconstruction of 198 frames with preceding
  recorded joints/environment classifies 64 NEAR and 134 FAR, clearance
  0.042680–0.139159 m. Three initial frames lack preceding samples and are
  skipped. All 24 reconstructed frames before 5.3 s are NEAR. Later live
  diagnostics include clearances 0.097004, 0.098806, 0.100093 and 0.099723 m,
  straddling the unchanged 0.10 m margin. Capsule endpoints also vary; the
  recording alone cannot separate actual human motion from measurement noise.
- A retreat must be re-certified against current geometry and capture expiry
  in _human_margin_guard and execution preparation. READY does not guarantee
  RELEASE/EXECUTE. UNSAFE can also result from expiry inside a geometric
  check, so its label alone does not prove a geometric collision.
- Target is constant during this bag but is 0.042805 m higher than baseline;
  initial joint configurations also differ. The 0.5 s bag contains invalid
  and empty detections absent here. Thus this is evidence of live retreat
  execution with 1 s, not a controlled proof of a lower stop/replan rate.
- Planning is still EXECUTING at bag end. The same live log reports COMPLETED
  10.884215 s afterward, with intervening EMPTY detections; that later outcome
  is outside bag coverage and does not prove completion near an observed human.

Evidence: test/results/recorded_stuck_1s_20261116_174406_diagnosis.json stores
bag hashes, baseline/trial statistics, status timelines, Motion acknowledgements,
coverage limits and eight timing matches. Recommended next discussion: profile
retreat search, repeated certification and STOP/RELEASE preparation before
choosing a longer timeout or changing the recovery algorithm. Preserve collision
checks and distinguish computational expiry from changing geometry.

### 2026-11-16 — Prepare the approved 1 second simulation trial

Status: User approved testing 1.0 s first after the recorded-scene comparison.
Planning's human_avoidance.data_timeout in src/cra/config/robot.yaml changes
from 0.5 to 1.0 s. PlanningNode passes the same value to HumanCapsuleMonitor
and CollisionFilter and now prints it in the initialization log.

- Keep the 0.10 m human margin, 0.005 m static-obstacle margin, existing
  GNG/A*, capture-based expiry, collision validation and Motion checks.
  execute_path=True and simulation_only_ignore_unknown=True remain configured.
- The separate upstream Vision reconstruction data_timeout remains 0.5 s;
  it checks input freshness before publishing geometry. Invalid geometry
  still produces VISION_INVALID regardless of Planning's longer deadline.
- The installed CRA robot.yaml is a symlink to source in this checkout.
  Running nodes still need restarting to reload it. Rebuild cra/planning
  before restarting to include the updated initialization diagnostic.
- Controlled procedure: keep one human and the target stationary, record
  the same seven topics with a unique cra_stuck_1s timestamped bag name,
  and compare expiry aborts, retreat/release decisions and Motion status
  with cra_stuck_static_20261116_170942. Preserve the baseline bag.
- This prepares a live experiment; no robot launch, Motion command or
  end-to-end execution is performed by this change.
- Verification: source and installed YAML load 1.0 s with unchanged margins;
  Python/JSON parsing and git diff whitespace checks pass. Three existing
  offline regressions pass (monitor timeout reasons, snapshot copies retaining
  capture expiry, and STOP on expiry before release). PlanningNode passes
  Flake8 with max-line-length=99. In cra-humble, colcon build --packages-select
  cra planning completes successfully for both packages.

### 2026-11-16 — Diagnose the complete 17:09:42 recorded stuck scene

Status: Latest rosbag inspected read-only; recorded geometry reconstructed and
offline deadline controls completed. No rosbag play, ROS publishers, Motion
commands, runtime algorithm changes or timeout/configuration edits were made.

Input and recording quality:

- Bag: /tmp/cra_stuck_static_20261116_170942 in cra-humble, 23.161177 s,
  4,827 messages, with all seven requested topics. Human geometry is present:
  308 messages consisting of 243 valid observed, 62 valid empty and three
  invalid messages. All human frames are base_link. The earlier recording
  /tmp/cra_stuck_static lacked human geometry and is not used here.
- Observed geometry age at rosbag receipt: median 0.241697 s, empirical p95
  0.261459 s, maximum 0.294602 s; none exceeds 0.5 s at the recorder. This is
  not necessarily the age when the Planning callback consumes the message.
- The target remains unchanged. Measured joint positions are exactly constant
  through the initial 9.3 s, then Motion starts and stops repeatedly. The sole
  recorded Motion failure value is NONE. Empty detections begin about 18.474 s
  into the bag; the whole recording is not an immutable human scene.

Recorded geometry reconstruction:

- Deserialize the actual CDR messages without initializing ROS. Match each
  observed frame to prior joint/target/obstacle samples by recorder time. Undo
  the marker's CRA/base transform and any XML local geom transform before
  setting mocap bodies; reconstructed marker positions agree within 1e-8 m.
  Both recorded static obstacles are removed; the recorded target box remains
  collidable. Installed PlanningManager/PathPlanner, robot.yaml and model XML
  match source hashes for this analysis.
- Freeze capture lifetime only for offline geometry inspection. The actual
  model/checker classifies 49 of 243 observed frames NEAR and 194 FAR using
  contemporaneous measured joints. Clearance ranges from 0.096246 to 0.698764 m.
- Eight selected frozen snapshots were examined: four NEAR snapshots admit
  certified two-point retreats reaching about 0.1050–0.1053 m clearance; four
  FAR snapshots admit normal graph paths (9/9/9/8 waypoints). All certificates
  use the unchanged 0.10 m human margin and the existing algorithms.
- These are feasible frozen-scene candidates, not commands for execution on
  old geometry. They demonstrate that the selected latest recorded scenes do
  not require a new geometric search method just to produce a candidate.
  They do not replay the older NO_MONOTONIC_STEP pose from the previous log.

Direct timing evidence:

- Joint pose and computed clearance uniquely match 15 observed reports from
  the live Planning log to recorded frames during the initial stationary-robot
  interval. For the first match (clearance 0.099016917 m), capture time is
  1794823782.920744151, bag receipt 1794823783.121213220, the relevant OBSERVED
  transition 1794823783.186203974, and diagnostic output 1794823783.422424803.
  Thus age increases from 0.265460 s at OBSERVED to 0.501681 s at diagnostic
  output over about 0.236221 s of the timer decision. The associated retreat
  diagnostic is HUMAN_DATA_UNKNOWN. Its frozen geometry admits a retreat.
- Nine matched diagnostic outputs already exceed 0.5 s of capture age. Matching
  by pose/clearance is evidence within this recording, not a globally unique
  frame identifier. OBSERVED transitions are not emitted for every frame update;
  only relevant transitions may be interpreted as processing-start bounds.

Counterfactual manager checks on four recorded snapshots:

- Run the actual PlanningManager.step() offline from an owned STOP with fresh
  measured feedback, recorded capsules/poses, and recorded receipt age. For two
  FAR cases, supply the already geometrically certified candidate and exercise
  the real preparation/revalidation/release gate. For two NEAR cases, exercise
  the real margin-retreat preparation. No returned action is published.

| Budget | Retreat preparation (2 scenes) | Normal-path release gate (2 scenes) |
| --- | --- | --- |
| 0.5 s | 0/2, HUMAN_DATA_UNKNOWN | 0/2, DATA_EXPIRED |
| 1.0 s | 2/2, HUMAN_RETREAT_READY | 2/2, RELEASE decision |
| 2.0 s | 2/2, HUMAN_RETREAT_READY | 2/2, RELEASE decision |

- At 0.5 s, manager steps abort at approximately 0.5001–0.5009 s of total
  input age. With 1/2 s budgets, the two retreat preparations take about
  0.45–0.65 s and the normal-path gate takes about 0.35–0.46 s in this run.
  Diagnostic timing records show the normal gate checking the remaining path
  in the guard, in execution preparation, and again before RELEASE.
- This is one timing run per selected scene/budget, not a success-rate study
  or live Motion comparison. Inputs use recorder receipt age and omit extra
  waiting before the real Planning callback. CPU load differs from the original
  run. A 1 s budget passing this gate does not prove the full live handshake or
  moving-human execution is safe or will always succeed.
- Three invalid input messages and changing/empty human geometry are separate
  conditions that cannot be repaired merely by increasing the age limit.

Artifacts and next decision:

- test/results/recorded_stuck_20261116_170942_diagnosis.json stores bag hashes,
  message statistics, selected recorded capsules/poses and candidate paths,
  timestamp matches, counterfactual decisions and phase timings. The adjacent
  recorded_stuck_20261116_170942_deadlines.csv contains the 12 manager decisions.
  Consistency checks cover all 243 frame reports, eight frozen-scene candidates,
  15 unique log matches and the 12 deadline controls.
- The initial attempt to reconstruct every frame by repeatedly loading a new
  MuJoCo model failed while loading an STL. Reusing one model completed the
  analysis; final results come only from the completed run.
- Discuss a controlled 1 s simulation trial versus reducing repeated validation
  cost/scheduling delays before changing runtime behavior. The latest evidence
  supports investigating computation/freshness first. Runtime remains 0.5 s;
  the latest four selected frames show no added acceptance from 2 s over 1 s.

### 2026-11-16 — Compare 0.5, 1.0 and 2.0 second human-data budgets

Status: The user-approved offline comparison is complete. Runtime robot.yaml
still specifies data_timeout=0.5. No runtime algorithm, safety margin, execution
setting or ROS node was changed or started by this experiment.

Method:

- Added test/compare_human_timeouts.py, a manual diagnostic script using the
  actual PathPlanner, graph, MuJoCo model, CollisionFilter and HumanCapsuleMonitor.
  It reuses the stationary synthetic detour fixture. This is geometric planning,
  not Motion execution, a live sensor replay or a human-safety validation.
- Compare timeouts 0.5 / 1.0 / 2.0 s at controlled initial capture ages
  0.0 / 0.49 / 0.75 / 1.5 / 2.1 s. Each of 15 combinations runs three times
  (45 trials), shuffled with seed 42, with a cold planner per trial.
- Both the monitor and collision-checker deadline use the selected timeout.
  Message capture timestamps and the remaining absolute lifetime are retained;
  the computation clock runs normally during planning. Capture ages are chosen
  experimental inputs, not measured latency from the user's live log.

Accepted paths / trials:

| Initial capture age | Timeout 0.5 s | Timeout 1.0 s | Timeout 2.0 s |
| --- | ---: | ---: | ---: |
| 0.00 s | 3/3 | 3/3 | 3/3 |
| 0.49 s | 0/3 | 3/3 | 3/3 |
| 0.75 s | 0/3 | 3/3 | 3/3 |
| 1.50 s | 0/3 | 0/3 | 3/3 |
| 2.10 s | 0/3 | 0/3 | 0/3 |

Interpretation and checks:

- All accepted paths are identical (same byte-level path digest), with
  11 waypoints and joint-space length 6.293658 rad. Increasing the budget did
  not find a different route. Every rejected trial reports HUMAN_VISION_UNKNOWN.
- On fresh data, mean search-plus-validation time is 0.125870 / 0.132982 /
  0.128827 s for the three budgets. The larger budgets are not a speedup.
  With capture age 0.49 s and budget 0.5 s, search aborts after about 0.010121 s.
- Three extra fresh-data controls use identical 0.06 m starting clearance.
  The ordinary start remains invalid under the unchanged 0.10 m margin for all
  budgets. The existing dedicated retreat succeeds for each, reaching
  0.106405 m; its measured times are about 0.150–0.153 s. This does not reproduce
  the live NO_MONOTONIC_STEP case or prove that a larger timeout can solve it.
- Result consistency checks verify 45 detour rows, the expected acceptance
  matrix, identical successful paths, the near-scene constraints and unchanged
  hashes of the input configuration/model/graph and planning source files.
  The script passes Flake8 (99-character limit) and syntax checks.
- Results are stored in test/results/human_timeout_comparison_2026-11-16.json
  and the corresponding .csv. JSON includes controlled geometry, source hashes,
  per-trial ages/timings/failure details and the near-scene controls. These are
  reproducible experimental observations, not a general success-rate estimate.

Repeat inside cra-humble from the workspace root:

```bash
PYTHONPATH=src/planning:src/simulation python3 \
  src/planning/test/compare_human_timeouts.py --repeats 3 \
  > /tmp/cra_timeout_comparison.json
```

Conclusion: 2.0 s permits planning with older stationary data in this control,
including the selected 1.5 s input age. It does not repair missing graph routes,
retreat restrictions or tracking faults. No timeout is selected for live use;
that decision still requires measured live capture ages and moving-human tests.

### 2026-11-16 — Diagnose stuck behavior before selecting another algorithm

Status: Log evidence and stationary offline controls completed. No runtime
algorithm, margin, timeout, ROS interface or execution behavior was changed in
this diagnostic task. Exact replay of the live human geometry remains pending.

The user selected proof of the stuck cause before algorithm selection. The
new log confirms Visualisasi OK, GNG publication and six robot meshes, so the
earlier visualization-package startup failure is resolved in that live run.

Live evidence (1,357 pasted lines):

- All 50 PathPlanner NO_PATH messages cite human-data expiry: 42 during graph
  search and eight during final validation. There are zero messages stating
  no collision-free graph connection. This establishes the reported abort
  reason; it does not prove that every live scene has a feasible route.
- One traced sequence starts with a 10-waypoint trajectory at epoch
  1794818403.402302. At +1.497 s, HUMAN_VISION_UNKNOWN stops Motion. At +2.101 s,
  HUMAN_TOO_CLOSE stops motion with about 0.071 m clearance. At +2.344 s, the
  retreat reports NO_MONOTONIC_STEP at 0.059914 m and becomes
  HUMAN_RETREAT_BLOCKED. At +5.904 s the snapshot has motion=STOPPED,
  fault=False and stop_owned=True. This is an owned safety stop, not a latched
  Motion fault. Later, A* returns a candidate which expires during validation.
- At +21.292 s a snapshot still retains 11 path points but records
  human_validation_pending=True / DATA_EXPIRED. Retaining a route does not
  authorize execution on expired geometry. A later seven-waypoint plan becomes
  ready and returns to the release/execution sequence.
- Across 27 collision reports in the near-stationary pose interval, the largest
  joint range is about 0.000209 rad while reported clearance varies from
  0.037573 to 0.172027 m. This shows changing perceived geometry relative to a
  nearly stationary robot. The log cannot distinguish actual human movement
  from sensor/reconstruction noise.
- There is also one independent STALLED failure on joint5; it does not explain
  the traced fault=False / HUMAN_RETREAT_BLOCKED interval.

Controlled offline proof in the existing cra-humble container:

- Actual source PathPlanner, A*, MuJoCo model, graph and CollisionFilter were
  used. No ROS nodes, publishers, robot commands or live-world changes were
  created. The scenarios use explicitly synthetic stationary capsules from the
  existing regression fixtures; they are not a replay of the user's scene.
- On a fixed detour scene, the unobstructed reference path has nine waypoints
  and joint-space length 4.542672 rad. The capsule blocks that reference, while
  the existing A* finds a valid 11-waypoint route of length 6.293658 rad. Minimum
  sampled human clearance is 0.100438 m with the unchanged 0.10 m margin. This
  proves the existing implementation can find a longer route in this control.
  Its small clearance reserve is not evidence of robustness to live noise.
- With geometry held identical, a controlled clock advance from 100.0 to
  100.6 s after successful geometric validation changes acceptance to
  HUMAN_VISION_UNKNOWN. Refreshing the identical geometry accepts the same
  detour again. This isolates time validity from geometric path existence.
  Frozen capture time is used only for this offline geometry control.
- A separate unfrozen-clock comparison uses the same stationary capsule and
  cold planners. Capture age 0.0 s accepts a route in 0.122661 s. A controlled
  capture age of 0.49 s leaves 0.01 s and aborts after 0.010141 s with
  HUMAN_VISION_UNKNOWN. These ages are experimental inputs, not measurements
  of capture latency in the user's live run.
- In a second synthetic scene, clearance 0.060000 m rejects the ordinary start
  under the normal margin, but the existing certified retreat reaches
  0.106405 m. Keeping that scene identical while expiring its data rejects the
  retreat with HUMAN_DATA_UNKNOWN. This does not reproduce the live
  NO_MONOTONIC_STEP pose.
- The logged NO_MONOTONIC_STEP joint pose satisfies joint limits and is valid
  with the logged target/default static obstacles when human checking is
  explicitly disabled for an isolated static-only baseline. This baseline is
  not evidence of human avoidance or of a valid route through the live scene.

Timing mechanism and limits:

- Source inspection inside the container confirms rclpy.spin(node) uses its
  global SingleThreadedExecutor. PlanningNode calls manager.step() synchronously
  in its timer callback, so that work prevents that executor from running the
  next Vision subscription callback until it returns. This is a verified
  scheduling mechanism, but its contribution versus sensor/reconstruction
  latency in the live trace has not been measured.
- Freshness requires capture age at planning start plus computation/validation
  time to stay within 0.5 s. A stationary human does not make old data fresh.
- Neither the pasted log nor the actual per-node logs store the full capsule
  endpoint arrays. Thus the exact live NO_MONOTONIC_STEP case cannot yet be
  reproduced, and no conclusion about a coordinated-joint alternative is made.
- A read-only ROS graph listing attempt returned 'sequence size exceeds
  remaining buffer', so no live scene capture was obtained in this task.

Local artifacts:
/tmp/cra_stuck_log_evidence.json, /tmp/cra_stuck_proof.py,
/tmp/cra_stuck_proof_results.json, /tmp/cra_stuck_realclock.py and
/tmp/cra_stuck_realclock_results.json. These are temporary diagnostic artifacts.

Next experiment, with the user participating: record 10–15 seconds while the
robot is stuck, keep the human and target stationary, and record /joint_states,
/vision/human/geometry, /simulation/target_marker, /simulation/obstacles,
/planning/status, /motion/execution_status and /motion/failure_reason. Preserve
ROS message timestamps and record the running configuration. Replay geometry
offline before choosing a different retreat/search method; do not relax live
freshness or margins to make an offline candidate executable.

### 2026-11-16 — Restore installed Planning visualization package

Status: Packaging correction implemented and verified with an isolated Python
package build. ROS launch and RViz display recovery remain unverified here.

The supplied launch log identifies an import-time crash, before PlanningNode
initializes: ModuleNotFoundError: No module named 'planning.visualization'.
The process exits with code 1. Simulation and Motion keep running, but Planning
cannot send trajectories or create the robot/GNG visualization publishers.
This log therefore does not show a replanning or tracking failure.

Confirmed cause and correction:

- setup.py uses setuptools.find_packages(). The visualization directory lacked
  __init__.py, so discovery omitted planning.visualization and its visualizer.
  Source-tree imports can work as a namespace subpackage, masking this omission
  in earlier source-based headless tests.
- Added planning/visualization/__init__.py. Package discovery now includes
  planning.visualization. No planning, collision, Motion or RViz behavior was
  changed.
- Added test/test_package_installation.py: copy the actual setup/package source
  into a temporary directory, build with setup.py build_py, and check module
  discovery using isolated Python (-I) with only that build directory added.
  This checks packaging without importing ROS or using the source PYTHONPATH.

Verification:

- The new packaging test fails before the fix because the built visualizer
  file is missing, and passes afterward. Logs:
  /tmp/cra_package_installation_before.log and
  /tmp/cra_package_installation_after.log.
- Python syntax and 99-character line-length checks passed. ROS/colcon and
  Flake8 are unavailable in this environment; no live startup or full existing
  test-suite result is claimed for this packaging-only correction.
- The first command in the supplied log, rviz:=rviz, is invalid for the boolean
  launch condition. The next attempt, rviz:=true, correctly starts RViz; its
  missing Planning markers are consistent with the Planning import crash.

Rebuild planning inside the ROS container, source install/setup.bash, verify
that planning.visualization.planning_visualizer imports there, and relaunch
with ros2 launch cra cra.launch.py rviz:=true. No build/install deletion or
collision-setting changes are needed for this correction.

### 2026-11-14 — Revalidate the remaining route and retain plans across Vision expiry

Status: Implemented; tested headlessly with real-model collision checks and a
MuJoCo dynamics probe. ROS 2 transport, C++ execution and live Kinect behavior
remain unverified. The pre-change development snapshot was pushed to origin/main
as f84d306 before this behavior change.

Evidence and scope:

- The supplied 1,477-line live excerpt contains 67 PATH_READY transitions but
  eight trajectory sends, 48 HUMAN_PATH_BLOCKED transitions, and repeated
  OBSERVED/STALE_MESSAGE changes. Graph-search expiry appears 23 times and
  final-validation expiry 15 times. These counts are status/log events, not
  a measured false-positive rate or number of distinct physical obstructions.
- Previously, every changed capsule snapshot revalidated the complete stored
  execution path, including completed segments. The same false return could
  mean a blocked route or human data expiring during validation. Under the
  simulation UNKNOWN policy, a recently observed human could disappear from
  checks and permit a replacement path, which was rejected when OBSERVED
  returned. This made path reuse and failure diagnosis unreliable.

Changes:

- PlanningManager records the next internal waypoint in order from fresh,
  finite measured joint feedback. Only the next waypoint can be recognized;
  progression never searches for the nearest future waypoint across the route.
  A short measured-motion segment can recognize passage between samples, using
  Motion's existing joint tolerance (0.02 rad), read by PathPlanner from robot.yaml.
  Missing/discontinuous feedback keeps the old progress conservatively. The
  tolerance is not applied to human or static collision clearance.
- Human updates validate current_q -> remaining waypoints. The entire current
  robot is still checked for proximity every guard cycle, and every remaining
  state/edge retains the existing CollisionFilter checks. A completed Motion
  validates the measured state rather than its historical trajectory. Small
  safe human movements keep the existing trajectory; a genuinely obstructed
  remaining route still requests STOP and normal GNG/A* replanning.
- Once a human is observed, expired/invalid data requests STOP and retains the
  route for fresh validation; it cannot trigger a search with humans disabled.
  A validated fresh EMPTY can still clear this context in the configured
  simulation experiment. Initial UNKNOWN/EMPTY behavior remains available via
  simulation_only_ignore_unknown=True; execute_path=True is unchanged.
- Following STOPPED and fresh measured feedback, a valid retained remainder is
  rebased to the stopped pose, prepared and fully revalidated, then sent through
  the existing RELEASE -> IDLE -> EXECUTE handshake without another A* search.
  STOP cancels Motion's trajectory: reuse sends a new validated remainder, it
  does not resume a canceled command blindly. Feedback waiting preserves the
  plan; target changes discard it. Faults and external STOP remain protected.
  A zero-length remainder at the measured goal reports GOAL_REACHED and keeps
  STOP owned instead of sending an invalid zero-length trajectory.
- Expiry during clearance queries, path validation, execution preparation or
  final validation reports HUMAN_VISION_UNKNOWN / DATA_EXPIRED. Geometry query
  exceptions report HUMAN_VALIDATION_ERROR / COLLISION_CHECK_ERROR; revision
  changes report HUMAN_VALIDATION_PENDING / ENVIRONMENT_CHANGED. Genuine route
  rejection records PATH_COLLISION. Dedicated human-margin retreats still need
  their own fresh monotonic certificate; they are not reused as ordinary paths.
- The five-second Planning snapshot adds human_validation_pending,
  path_validation and next_waypoint to make the distinction visible.

Verification:

- 109 Planning/geometry/visualization tests and six simulation control tests
  pass (115 total). Twenty added cases cover progress through real graph paths,
  future loops, missed/invalid feedback, small safe capsule changes, completed
  paths, expiry at multiple validation stages, plan reuse after STOP/feedback,
  invalidation by a real obstruction/new target, static blockers after EMPTY,
  late RELEASE acknowledgments and unowned STOP. One previous test was updated
  to require waiting for fresh data instead of planning past a known human
  during UNKNOWN. Logs: /tmp/cra_remaining_route_all_tests.log and
  /tmp/cra_remaining_route_simulation_tests.log.
- A real-model case has a human capsule obstructing the original 10-waypoint
  path but not its suffix from waypoint 5. After ordered measured passage,
  Planning returns NONE/EXECUTING and makes zero new A* calls. Introducing the
  same obstruction before passage still stops and the existing detour test
  certifies a replacement path. The fresh-data resume case also makes zero new
  A* calls while preserving the original target and validating the remainder.
- An actual simulation-callback/physics probe follows that graph route with a
  reproduced 200 Hz linear Motion control law: all eight internal waypoints
  are recognized in order, ending at next_waypoint=9, peak reference error
  0.032226 rad, zero contacts. /tmp/cra_measured_progress_probe.json records it.
  Progress uses measured geometry, not a new C++ waypoint-acknowledgment topic.

No capsule filtering, margin reduction, A* cooldown, executor/thread change or
freshness-timeout increase was introduced. Lost/slow Vision can still require
stops; this change prevents unnecessary path destruction/search, not data loss.
If the next waypoint is not recognized, checks keep the older path prefix.
Live experiments must measure remaining false stops and data age. Rebuild
planning, source the overlay and restart CRA before comparing the new logs.

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
