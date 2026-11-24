# Tracker admission in the installed driver

`tracker_enqueue_wait_ms` is a read-only ROS integer parameter selected at
startup. The installed driver accepts:

| Value | Behavior |
| --- | --- |
| `-1` (default) | Existing blocking SDK enqueue, with infinite wait. |
| `5` | Wait up to 5 ms for tracker admission; skip that tracker submission on timeout. |

A timed-out submission does not increment pending tracker results. RGB/depth
publication continues through the existing capture path. SDK FAILED still
throws into the existing worker error/shutdown handler. Unexpected enqueue
failure in blocking mode remains fatal. No previous frame or pose is stamped
as new. Tracker output can be less frequent with the bounded policy; freshness
and pose accuracy are different properties.

The 0 and 10 ms variants remain available only in the BUILD_TESTING executable.
The comparison runner now uses the installed driver for blocking and 5 ms;
its summaries identify `driver_variant` and the binary hash. The driver's own
default remains blocking. CRA launch defaults come from `robot.yaml`; the
shipped CRA configuration selects 5 ms and enables RViz.

## CRA launch

Set defaults in `src/cra/config/robot.yaml`:

```yaml
rviz:
  enabled: true
vision:
  tracker_enqueue_wait_ms: 5 # -1 for blocking
```

CRA reads the installed YAML. Its symlink installation points to the source
configuration. Missing settings fall back to RViz disabled and blocking.
CLI arguments can override the YAML for one launch. The tracker argument is
forwarded through CRA and Vision into the driver as an integer. Sensor
FPS/resolution/calibration stay unchanged. From one host terminal:

```bash
docker exec -it \
  -u "$(id -u):$(id -g)" \
  -e DISPLAY="$DISPLAY" \
  -e ROS_LOG_DIR=/tmp/cra_main_tracker_ros_logs \
  -w /home/lvinz/living-lab/cra_ws \
  cra-humble bash -c '
source /opt/ros/humble/setup.bash &&
for pkg in msgs cra vision azure_kinect_ros_driver planning simulation motion; do
  source install/"$pkg"/share/"$pkg"/package.bash || exit
done &&
ros2 launch cra cra.launch.py
'
```

For blocking, set the YAML value to -1 and restart, or override the launch with
`tracker_enqueue_wait_ms:=-1`. For RViz off, use `rviz:=false`. The parameter
cannot be changed on an active driver; there is
no automatic policy switching. Close other camera launches first. The host
UID and writable log path preserve the previously verified X11 setup.

Expected startup line:

```text
Tracker admission: mode=bounded enqueue_wait_ms=5 (startup-only)
```

Every existing 5 s capture profile interval can also report
`Tracker admission profile: enqueue_wait_ms=5 sdk_busy_skips=...`.
The skip count describes SDK admission timeouts in that interval, not all
possible capture drops or the number of independent human observations.

Sensor FPS/resolution/mode, body smoothing, calibration, topics, Reconstruction's
0.5 s watchdog, Planning's 2 s timeout, human margin and motion policies are
unchanged. This option does not establish missing USB restart behavior or
human-obstruction avoidance that earlier trials did not exercise.

## Build and automatic checks

Inside the sourced workspace container:

```bash
colcon build --packages-select azure_kinect_ros_driver cra vision --symlink-install
ctest --test-dir build/azure_kinect_ros_driver --output-on-failure
python3 src/cra/test/test_tracker_admission_launch.py
```

The executable is installed to `lib/azure_kinect_ros_driver/node`, the location
used by ROS launch. The earlier generic `bin/node` destination could leave
ROS using a stale file under `lib`; the corrected install rule replaces that
file with the newly built executable.

Tests validate wait choices, SDK timeout/exception handling, pending-result
accounting, launch forwarding, and native startup under a shim that denies
camera access. Native startup is also checked against the installed binary.
These are automated checks, not a new live Kinect/MuJoCo validation of the
promoted driver. Earlier real-camera results remain scoped to their binaries.
