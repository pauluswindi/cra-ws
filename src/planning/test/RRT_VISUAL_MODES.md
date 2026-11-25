# RRT live and playback visualization

Choose `planning.rrt.visualization.mode: live` or `playback` in robot.yaml,
then restart launch. Default: playback, 10 Hz, 2 s animation plus 3 s history.
`visualization_enabled: false` skips trace capture. No Motion command waits for
animation, and displayed candidates never constitute safety certificates.

- Playback reveals the original recorded events and tree nodes in order; final
  candidate appears at the end. Actual search duration is labeled separately.
- Live sends cumulative bounded raw progress snapshots through a one-slot spawn
  queue. Sending never waits for the display; packets may be omitted. Separate
  parent FK data and a per-attempt cache project configurations without collision
  calls. Final results replace partial data. Very fast searches may show only
  the result. Live is not artificially slowed down.
- Blue: start tree; magenta: goal tree; yellow: latest recorded search step;
  purple: candidate path. Existing Planned path displays the actual Planning
  route separately. Direct start-goal success says DIRECT CONNECTION; no fake
  branches are created.
- Changed/expired scenes remain faded, labeled HISTORICAL SCENE and candidate
  only. Planning status is shown separately. Target changes clear the record;
  a new search replaces it. Original snapshot expiry is never rewritten.
- Current RViz defaults hide workspace graph clutter; enable its existing
  checkbox for comparison. RRT start/goal/final/status displays remain enabled.
  To show rejections, enable both YAML show_rejected and its RViz checkbox.

Configuration limits bound memory/serialization work: <=1024 nodes per tree,
<=4096 events, <=20 publications/s, <=30 s playback/history each. Defaults are
256/512/10; max_rejected remains 64. Omitted counts are shown. A terminal event
is retained even if event recording is full. Marker geometry is batched, and
live FK results are cached per attempt. Publication uses the existing 50 ms
observer timer, so rates above its cadence are not guaranteed. These are soft
CPU limits, not real-time scheduling guarantees. Projection is of five-joint
configurations into end-effector XYZ; tree edges are not Cartesian trajectories.

Validation: test_rrt_animation, test_rrt_visualization, existing RRT/Planning
regressions. Native headless probe supports --rrt-visual-mode live/playback and
--changing-human. It uses synthetic distant human geometry, real C++ Motion,
MuJoCo and ROS marker transport, not live Kinect or visual GUI inspection.
