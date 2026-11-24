# Tracker admission: disturbance during Motion execution

The timed FPS comparison remains available. These additional scenarios test
the observed STOP/recovery sequence after the **requested test target** starts
executing. Initial simulator motion does not trigger a disturbance. The harness
waits for warm-up, recent observed human geometry and IDLE/COMPLETED before
sending the target. The native probe also checks workspace occupancy, geometry
receipt freshness and the single simulation target subscriber.

Production camera settings, calibration, Reconstruction's 0.5 s watchdog,
Planning's 2 s timeout, collision margins and simulation policy are unchanged.
CRA launch now reads `vision.tracker_enqueue_wait_ms` from robot.yaml, shipped
as 5; the driver's own fallback remains -1 (blocking). This does not modify
GNG/A*, RRT, Motion or the legacy planner. The installed
driver supports startup parameter `tracker_enqueue_wait_ms:=5`; current
blocking/5 ms comparisons use that installed driver. The 0/10 ms variants still
require the separate experimental executable.

## Scenarios

- `--scenario vision-loss`: after target EXECUTING, stop forwarding depth to
  Reconstruction for 3 s. The driver, RGB, body tracker and raw depth continue
  running. This tests one missing input and its watchdog/recovery, **not** USB
  disconnection, SDK shutdown or camera restart. Stay visible throughout.
- `--scenario blockage`: give the human-obstruction cue after target EXECUTING.
  On STOPPED, ask the participant to clear the route while remaining visible.
  Use `--viewer` to see the robot's path. Only a corresponding
  HUMAN_TOO_CLOSE/HUMAN_PATH_BLOCKED status supplies planner-side disturbance evidence;
  participant movement or mere presence in the workspace does not prove it.
  For HUMAN_PATH_BLOCKED inspect the logged Path validation failure to confirm
  whether a human or another collision constraint rejected the route. The
  scenario verdict alone does not establish collider identity.

Both scenarios route depth through the native probe to `/tracker_trial/depth`;
only Reconstruction is remapped. The relay is pass-through outside the
controlled loss. It adds transport overhead, so compare blocking and 5 ms
with this same topology; do not interpret rates as the original FPS benchmark.
No capsule coordinates or camera image payloads are stored in the reports.

## Run from one host terminal

Close other CRA/Kinect launches first. The installed `robot.mode` must be
`simulation`. Use the host UID for X11 access instead of container root;
`bash -c` avoids reading root's login profile. A separate ROS log directory
and fresh output path avoid the earlier root-owned files.

```bash
docker exec -it \
  -u "$(id -u):$(id -g)" \
  -e DISPLAY="$DISPLAY" \
  -e ROS_LOG_DIR=/tmp/cra_tracker_motion_scenario_ros_logs \
  -w /home/lvinz/living-lab/cra_ws \
  cra-humble bash -c '
source /opt/ros/humble/setup.bash &&
for pkg in msgs cra vision azure_kinect_ros_driver planning simulation motion; do
  source install/"$pkg"/share/"$pkg"/package.bash || exit
done &&
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
python3 src/perception/vision/test/compare_tracker_admission.py \
  --mujoco --guided --scenario vision-loss \
  --modes blocking wait-5ms --target 0.50 0.10 -0.25 \
  --warmup 15 --duration 40 \
  --output /tmp/cra_tracker_motion_loss
'
```

For the separate human trial use `--scenario blockage --viewer` and a different
output directory, such as `/tmp/cra_tracker_motion_blockage`. Follow the live
instructions instead of the four timed FPS-comparison cues. If the path is too
short to obstruct, choose another target through `--target X Y Z` and repeat.
The regular planner validates that target and route; the harness does not slow
the controller, bypass collisions or extend the freshness limits.

The probe is built with:

```bash
cmake --build build/vision --target vision_stream_probe -j2
```

Run this in the sourced container after editing the C++ test probe. It is not
installed as a production node.

## Read results

`comparison.json` contains `trials[].scenario_result`, including event times:

- `passed`: the disturbance, STOP command, STOPPED, observed recovery geometry,
  RELEASE, a new trajectory, EXECUTING and both Motion/Planning COMPLETED were
  observed. An intermediate retreat completion alone is insufficient.
- `not_exercised`: prerequisites or disturbance evidence were absent, the robot
  completed before the disturbance, time was insufficient, or DDS ordering at
  an invalid/valid boundary was ambiguous. This is not a pass.
- `failed`: a required response/recovery was missing, or a release/new trajectory
  or execution was observed well inside a sustained invalid-data interval.
  Inspect `reason` and logs; failure alone does not establish a software cause,
  e.g. the person may still block recovery.

The 0.2 s receipt-order uncertainty in the evaluator only qualifies inference
from different DDS topics. It does **not** change a runtime safety timeout.
STOP versus STOPPED receipt order alone is not a failure. Reported deltas are
observer receipt differences, not sensor-to-actuator latency guarantees.
Status COMPLETED is not a measurement of Cartesian target accuracy. No result
certifies physical braking distance, pose accuracy during occlusion, all human
motions, or RRT fallback. A* may solve the route without RRT.

## Automated verification

Pure logic tests:

```bash
python3 src/perception/vision/test/test_tracker_admission_runner.py
python3 src/perception/vision/test/test_tracker_motion_scenario.py
```

In the sourced container, with domain 221 unused:

```bash
ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
python3 src/perception/vision/test/verify_motion_scenario_ros.py \
  --output /tmp/cra_tracker_contract

ROS_DOMAIN_ID=221 ROS_LOCALHOST_ONLY=1 \
python3 src/perception/vision/test/verify_motion_scenario_ros.py \
  --with-simulation --output /tmp/cra_tracker_physics
```

The first uses synthetic frames and mocked Motion/Planning statuses but the
real probe, depth relay and Reconstruction watchdog. The second uses real
Planning, C++ Motion and headless MuJoCo with synthetic frames/skeletons. Neither
is a Kinect/human experiment. Both refuse a domain with existing publishers
on the relevant camera/geometry/motion/target topics and clean up only their
own children.
