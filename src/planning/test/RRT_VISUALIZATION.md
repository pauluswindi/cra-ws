# RRT observation in RViz

GNG+A* remains primary. Current simulation YAML enables RRT as a fallback;
see [RRT_FALLBACK.md](RRT_FALLBACK.md) for execution gates and tests. The marker
layer remains diagnostic and never dispatches trajectories, STOP or RELEASE.
`mode: shadow` retains observation-only behavior. Mandatory margins, human
capsules and Motion safety checks are unchanged.

## Workspace graph display

The heavy progressive whole-arm graph display was removed at the user's
request. `Workspace graph - obstacle mask` now uses exactly the same stored
1000 GNG points and 4453 edges as `GNG - original`. It hides XYZ points/lines
inside human capsules and simulation boxes without FK or robot collision
queries. The original overlay defaults off so it does not cover the holes.
This is a visual mask, not evidence of whole-arm motion validity. See
[WORKSPACE_GRAPH_MASK.md](WORKSPACE_GRAPH_MASK.md) for behavior and diagnostics.
The previous filtered topics, publisher and spawned graph-filter worker no
longer run; historical test artifacts describe the earlier implementation.

## RRT algorithm and joint-space projection

`RRTConnectPlanner.plan()` samples five joint angles. `_extend()` selects the
nearest configuration, steps toward the sample, and calls the existing
`is_state_valid()` and `is_edge_valid()` before inserting a child. Goal-tree
edges are checked toward the goal root in execution order. `_connect()` repeats
extension until trapped or connected. `_finish()` certifies the assembled path.
A directly valid start-to-goal edge skips random exploration and shows two roots.

An accepted node represents the entire arm, not a Cartesian point obstacle.
RViz receives `FK(q)` in CRA `base_link`, using the same MuJoCo model, joint
ordering, base rotation and `FrameTransform` as the existing visualizer. The FK
function was extracted without changing its calculation. A line joining two
projected end-effector positions illustrates their joint-space relationship;
it is not the actual end-effector trajectory. A projected line crossing a
capsule is not itself a collision test.

## Lightweight live integration

No additional live ROS node or collision worker is created. The existing
shadow worker optionally records a baseline RRT search, then performs FK with
its private model/data after search and preference processing. It returns at
most 256 accepted nodes per tree, 64 rejected candidates and 512 diagnostic
events including a reserved terminal event. The final path is bounded by the
existing 3000-node search limit. Capping display data never caps the algorithm.

The parent builds markers once per result and republishes cached batches at
at most 5 Hz. It runs no RRT FK or collision queries in the marker callback.
Marker lifetimes are at most 0.6 s and never exceed the source snapshot's
remaining freshness. Projection time is separately reported in worker data.
No marker subscribers means no trace/FK work for a new search. Recording has
small CPU cost inside the existing cooperative search budget; this is not a
hard real-time guarantee or a guarantee of identical timing-limited outcomes.

RViz has independent MarkerArray checkboxes:

| Display | Topic | Default |
|---|---|---|
| RRT - Start Tree (blue) | `/planning/rrt/start_tree` | on |
| RRT - Goal Tree (red) | `/planning/rrt/goal_tree` | on |
| RRT - Final Path (green) | `/planning/rrt/final_path` | on |
| RRT - Rejected Candidates (orange; unknown gray) | `/planning/rrt/rejected_candidates` | off |
| RRT - Search Step and Status | `/planning/rrt/search_step` | on |

Human capsules continue to come from the existing Vision display/publisher.
No human geometry or Vision publisher is duplicated. State/edge rejection
reasons come from existing checker diagnostics: human collision, MuJoCo
contact, invalid joints, unknown geometry, expiry/revision changes or checker
errors. Generic checker rejection is not described as confirmed human collision.

The green line belongs to the **certified baseline RRT search snapshot**. It
does not show a clearance-preference replacement or a selected live Motion
path. Search trees may exist even when no final path is found. Root and parent
indices are retained; rejected candidates never enter accepted tree markers.

Each result retains the original scene revision and expiry. New human geometry,
UNKNOWN/stale observations, stale simulation obstacles, target replacement or
expiry clears the old display. Fresh receipt of identical human geometry does
not extend an old RRT result's certificate. Old trees are not revalidated in
live callbacks. Historical results remain available through offline replay.

The RRT coordinator searches **only after an eligible GNG+A* failure**, subject
to its existing feedback, human freshness, Motion-context and retry gates.
Turning on a checkbox does not launch a new planning attempt. A search that
ran without marker subscribers is not rerun just to populate RViz.

```bash
# Inside cra-humble, from the workspace after sourcing ROS/install:
colcon build --packages-select planning cra --symlink-install
source install/setup.bash
ros2 launch cra cra.launch.py rviz:=true
# Or use the existing console-only demo: ros2 run cra demo

# Another terminal, same ROS domain:
ros2 topic echo /planning/rrt/status --qos-durability transient_local --once
ros2 topic echo /planning/rrt_shadow
```

`planning.rrt.visualization_enabled: false` disables trace/projection
work while preserving the selected RRT mode. Unchecking all five RRT displays
also avoids trace work unless another marker subscriber is present.

## Deterministic offline comparison and replay

The recorder uses actual CRA CollisionFilter and the same start/goal in both
scenes. Human freshness is deliberately frozen only for this historical
geometry experiment; the 5 s search deadline uses real wall time.

```bash
PYTHONPATH=src/planning:src/simulation:src/planning/test \
  python3 src/planning/test/record_rrt_experiment.py \
  --seed 9 --output-dir /tmp/cra_rrt_replay_20261119
```

Observed results: without a human, a valid direct path and two roots; with the
synthetic capsule blocking the direct transition, a valid detour, 92 accepted
nodes and 64 recorded rejections (the display cap). The final path passes the
actual CollisionFilter again. These are geometric fixtures, not camera or
moving-human trials.

Use a separate domain so replay markers cannot mix with the live scene. In
terminal 1 (after sourcing ROS and install):

```bash
ROS_DOMAIN_ID=226 ros2 run planning rrt_replay \
  --file /tmp/cra_rrt_replay_20261119/with_human.json --rate 5
```

In terminal 2:

```bash
ROS_DOMAIN_ID=226 rviz2 -d src/cra/config/cra.rviz
```

Enable `RRT - Search Step and Status` and optionally rejected candidates.
Yellow points show the current sample/nearest/connection event; text names
the actual event and its reason. Replay reconstructs growth from recorded
parent indices and reveals the final path only at PATH_FOUND. If event history
was capped, the final frame explicitly jumps to the bounded completed
snapshot; omitted-event/node counts remain visible. Replay is labelled
OFFLINE HISTORICAL and never claims current-scene validity. Ctrl+C stops it;
the last frame otherwise holds. Repeating the command restarts playback.

For a presentation, run the two JSON files in turn with the same RViz view and
capture the window using the desktop screenshot/screen recorder. Keep the FK
projection/historical label and omission counts visible. Offline JSON retains
the synthetic capsule parameters but does not start a duplicate human marker
publisher. To include rendered human capsules, record/replay the existing
Vision capsule topic in the matching experiment scene. Do not overlay unrelated
live human data and describe it as the fixture's collision scene.

## Verification and limits

The status document and `results/rrt_visualization_20261119.json` record the
executed checks. They include actual MuJoCo geometry, analytic invalid
state/edge cases, parent certificates, shared FK versus stored `joint_eof`,
trace limits, replay ordering, stale/revision/target clearing and renderer
failure isolation. Native DDS probes cover real shadow marker delivery,
DELETE on revision changes, the workspace XYZ mask, and offline replay;
they receive no Motion commands. Build and the installed replay entry point
are checked. Probe scripts require domains 224, 227 and 226 respectively.

Normal A*/Motion timers are deliberately cancelled in live-node transport
probes. Native RViz rendering, interactive checkboxes, full-graph throughput,
live Kinect noise and dynamic avoidance while moving remain unverified.
