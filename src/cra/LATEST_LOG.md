# Latest CRA launch log

The normal `ros2 launch cra cra.launch.py` automatically records child-process
stdout/stderr in one file, replacing that file at the next launch. Configure it
in `src/cra/config/robot.yaml`:

```yaml
logging:
  enabled: true
  latest_file: /home/lvinz/living-lab/cra_ws/logs/cra_latest.log
  max_size_mb: 20
```

The separate `logs/` directory is ignored by Git. It avoids the root-owned
colcon `log/` directory, which the host UID cannot write. No camera settings,
node log levels, collision checks or Motion commands are changed. Existing
ROS/colcon logs and unrelated files are not deleted. This is a latest-only
CRA diagnostic file, not a replacement for every ROS logging subsystem.

## Runtime behavior

- File creation/truncation happens at launch execution, not description loading
  or `--show-args`. An advisory lock is acquired before truncating the file;
  a second overlapping launch cannot erase the active logger's file.
- stdout/stderr from included launches, including Planning, Motion, simulation,
  Vision, SDK diagnostics and RViz, is collected through `OnProcessIO`. Process
  start/exit metadata is included. Terminal output continues normally. Parent
  launch messages/errors that are not child-process output are not duplicated.
- The launch callback only enqueues chunks with `put_nowait`. A daemon thread
  frames lines and writes a buffered file, flushing approximately every 0.5 s.
  No image data or rosbag is recorded and no per-line fsync is used.
- The queue holds at most 256 chunks of up to 8 KiB (about 2 MiB of payload).
  Partial-line buffers are capped per process stream. If the queue fills,
  chunks are dropped rather than making launch wait on disk; the file reports
  `dropped_chunks_total`. Nonzero drops mean the saved output is incomplete.
- The file is limited to 20 MiB with the shipped setting. At the limit it is
  truncated in place and a marker explains that the earlier part of the same
  run was replaced. Only the newest portion is retained; no backup files pile up.
- CLI exit drains queued shutdown output and partial final lines, with a
  maximum 2 s join. Forced termination/power loss may lose buffered data. A
  file-open/write failure leaves terminal output running and reports a warning.
- `logging.enabled: false` disables the additional file and leaves its prior
  contents untouched. `ros2 run cra demo` remains console-only and does not
  activate the latest logger.

## Use

Keep the usual sourced-container command:

```bash
ros2 launch cra cra.launch.py
```

RViz and tracker defaults still come from the same YAML. To watch the file in
another terminal, use `tail -F /home/lvinz/living-lab/cra_ws/logs/cra_latest.log`.
The file is created by the user running the launch; it is not created by the
build or the automated tests. Logs can be read while the program is running.

## Verification

In the sourced CRA container:

```bash
colcon build --packages-select cra --symlink-install
python3 src/cra/test/test_latest_log.py
python3 src/cra/test/verify_latest_log_ros.py \
  --helper install/cra/share/cra/launch/latest_log.py \
  --output /tmp/cra_latest_log_verification.json
```

Tests cover replacement without deleting other files, active-file locking,
size cap, queue overflow, live flushing, fragmented UTF-8/ANSI output and final
partial lines. The native ROS test launches real C++ TF and a Python process,
checks two successive runs overwrite the same file, preserves console and
shutdown output, and checks disabled/console-only cases. It uses temporary
files and localhost domain 224, with no camera, robot or CRA control pipeline.
Whole-pipeline FPS/latency effects have not been measured.
