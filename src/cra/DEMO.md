# CRA display/demo mode

Build `cra`, source the workspace, then run inside the CRA ROS container:

```bash
colcon build --packages-select cra
source install/setup.bash
ros2 run cra demo
```

This starts the existing `cra.launch.py` pipeline with RViz enabled by default.
All ten saved RViz displays are checked: grid, measured robot, collision bounds,
human capsules, human skeleton, simulation obstacles, target, path, GNG nodes
and GNG edges. A checked display still needs messages on its configured topic.
More visible geometry can add rendering load.

The demo entry point configures launch logging before creating LaunchService:
file handlers go to `/dev/null`, and child process output stays on screen.
A global launch prefix execs each existing ROS executable with
`--ros-args --disable-external-lib-logs`, disabling the ROS external file sink
while retaining console output and rosout. It preserves process signals,
component container arguments and camera respawn. No log directory is created
by the demo logger. Temporary ROS parameter files are not log files.

Do not append `tee` or redirect output to a file when logs should not be saved.
The launcher does not start rosbag recording or delete older logs. Programs
explicitly configured to write their own files are outside ROS log control.
Current CRA launches use ROS nodes without custom process prefixes.

Existing launch arguments can be forwarded, for example:

```bash
ros2 run cra demo rviz:=false
ros2 run cra demo goal_index:=100
```

Stop with Ctrl+C. The normal diagnostic launch remains available:

```bash
ros2 launch cra cra.launch.py rviz:=true
```

Motion, collision checks, camera settings and experimental execution flags are
identical in both modes. A separate entry point is needed because the ordinary
`ros2 launch` CLI initializes file logging before loading the user's launch file.

Verification on 2026-11-17: cra build and installed executable help pass. Run
`python3 src/cra/test/test_demo_console.py --demo install/cra/lib/cra/demo` in
the sourced ROS container to reproduce the hardware-free comparison: actual
C++ tf2_ros and Python rclpy processes produce three log files in normal mode,
none in demo mode, retain console INFO/WARNING, and shut down cleanly. The
installed RViz YAML has all ten displays enabled. Full CRA/graphical demo and
camera/robot behavior were not exercised by these checks.
