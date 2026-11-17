# Kinect capture timeout and worker shutdown

Implemented on 2026-11-17 after user approval. The supplied 08:19 run starts
K4A successfully, then stops on a capture timeout about 19 seconds later.
The SDK C++ wrapper returns false for TIMEOUT and throws for FAILED; these
outcomes must not share an unconditional fatal-timeout branch. Queue-drop
messages after shutdown do not establish the original cause of the timeout.

## Runtime behavior

- `capture_max_consecutive_timeouts` defaults to 5 and must be positive.
  Set it through ROS parameters at startup; live parameter changes are not
  implemented. The fifth consecutive timeout stops the stream. A successful
  capture resets the counter, so isolated misses do not accumulate.
- A timeout clears the capture and skips publication. No previous image or
  newly stamped substitute is published. Downstream freshness checks remain
  responsible for identifying missing human observations.
- The existing initial wait remains 4 seconds. Later waits remain
  `1000 * 5 / fps` milliseconds: 166 ms at 30 FPS. Five regular misses consume
  approximately 0.83 seconds of SDK waiting; five initial misses consume about
  4.664 seconds. These are not wall-clock deadlines: scheduling adds delay.
- A real SDK failure still stops the stream; it is caught at the worker boundary
  instead of escaping the thread and terminating the process unhandled.
- Cleanup first sets atomic `running_` false and stops camera/IMU producers and
  the tracker, interrupting SDK waits, then joins only started workers. Handles
  remain alive until workers finish. This also supports partial startup failure.
- Startup refuses to create a tracker when no device or recording is open.

FPS, image resolution, topics, the 2 s Planning data timeout, the 5 cm human
margin and existing execution flags are unchanged. Existing launch respawn
still applies after persistent stream failure. This patch does not repair a
physical USB failure or establish why the original capture was delayed.

## Verification

In `cra-humble`, from the workspace after sourcing `/opt/ros/humble/setup.bash`:

```bash
colcon build --packages-select azure_kinect_ros_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select azure_kinect_ros_driver --return-code-on-test-failure
colcon test-result --test-result-base build/azure_kinect_ros_driver --verbose
```

Twelve C++ unit tests cover timeout/reset policy, interrupt-before-join ordering,
partial workers and exception containment. Four native startup cases cover no
device, denied device open, invalid retry count and recording-open exception.
The native suite uses a test-only SDK shim that denies every camera open and
is never installed. It also passed against the installed ROS executable.
See `results/startup_installed_20261117.json` and
`results/stream_recovery_20261117.json` for scope and artifact hashes.

These checks do not exercise real camera capture recovery, tracker shutdown
under live load, or the complete Vision/Planning/Motion pipeline.

## Next live experiment

Stop the previous CRA launch and, inside the CRA container, run:

```bash
source install/setup.bash
ros2 launch cra cra.launch.py rviz:=true 2>&1 | tee /tmp/cra_camera_retry_$(date +%Y%m%d_%H%M%S).log
```

Expected startup: `Capture retry limit=5, regular wait=166 ms`.
If a transient miss occurs, expect `Capture timeout (.../5)` followed by
`Capture resumed after ... timeout(s)` while fresh images resume. If all five
attempts time out, expect `No capture after 5 consecutive timeouts` followed
by cleanup ending in `Camera stream workers stopped`, and launch respawn.
Check actual shutdown completion as well as image/skeleton continuity.
If persistent failure remains, inspect SDK/USB evidence and then discuss a
separate 15 FPS comparison; do not mix that experiment with this retry change.
