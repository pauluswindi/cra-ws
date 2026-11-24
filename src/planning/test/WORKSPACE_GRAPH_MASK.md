# Lightweight workspace graph obstacle mask

`Workspace graph - obstacle mask` replaces the retired whole-arm filtered graph
display. It uses the same `positions` and `edges` arrays from
`workspace_graph.npz` as `GNG - original`: 1000 points and 4453 edges in this
checkout. Node coordinates, connectivity, graph assets and Planning inputs
remain unchanged. There is no new graph generation or graph-filter process.

## Meaning of the mask

All XYZ nodes inside a human capsule or active simulation box are hidden.
All straight graph edges intersecting those volumes are hidden too, including
edges with both endpoints outside. Hidden elements return when an obstacle
moves away. Capsule end spheres, tangency, zero-length segments and rotated
boxes are included. Actual capsule radii and box sizes are used without adding
a display safety margin. Existing Planning margins remain unchanged.

This is **display-only XYZ occlusion**, not whole-arm collision validity. A
visible end-effector point can still correspond to an arm configuration whose
elbow collides. A* and CollisionFilter retain their complete state/edge checks,
path certification and Motion interfaces. No mask is passed back to planning.
RRT visualization is unchanged.

The display reads validated human capsules in `base_link` from Planning's
existing snapshot. Target and `obstacle_*` boxes use the existing model's
compiled geom poses/sizes, transformed into the same CRA base frame. It does
not call MuJoCo FK, collision queries or alter the model/data. Removed boxes
with disabled contact masks are ignored. Current simulation assets are boxes;
an unsupported active geometry causes a display-only error, not silent safety
classification.

## Update behavior and load

The static graph endpoints and bounding boxes are cached once. Each changed
scene first uses bounding-box rejection, then batched NumPy finite-segment
distances for nearby capsules. Rotated boxes reuse `segment_box_distances()`.
Only nodes/edges are masked; nothing is dynamically sampled in joint space.

The existing Planning cycle polls freshness/subscriber state. No subscriber
means no mask calculations. Identical geometry performs no distance queries;
changed geometry that produces the same visibility sends no graph markers.
Only changed masks/status produce a MarkerArray. Human geometry is compared
directly as well as by scene revision, including when the simulation UNKNOWN
policy temporarily disables human checks. No geometry rounding or delayed
collision decisions are introduced in Planning.

Markers persist so an unchanged graph does not need continuous republishing.
The graph keeps its original green/orange colors and opacity even at startup
without a camera. UNKNOWN/expired human data removes only the human mask;
independently fresh target/simulation boxes are still masked. Conversely,
stale simulation poses remove only box masks while fresh human capsules still
apply. Missing inputs are noted in the status text as partial geometry, not
interpreted as evidence that the robot is safe. When no geometry is available,
the complete stored graph remains visible. Planning's camera-loss/STOP policy
is unchanged. If the main executor is blocked, the last display snapshot can
remain until callbacks resume; this is never a live safety certificate.

## Run and RViz settings

Inside `cra-humble`, from the workspace after sourcing ROS:

```bash
colcon build --packages-select planning cra --symlink-install
source install/setup.bash
ros2 launch cra cra.launch.py rviz:=true
```

Dataset defaults now come from `workspace.path` and `gng.graph_path` in
`src/cra/config/robot.yaml`; main, Planning and Simulation launches expand `~`
and share those defaults. CLI path overrides remain optional.

Restart an existing CRA launch to replace its old Python node. Reopen the
installed `cra/config/cra.rviz` if RViz still uses a saved older layout.
`Workspace graph - obstacle mask` defaults **on**. `GNG - original` defaults
**off**; enabling the original overlay draws the hidden elements again, so
leave it off when inspecting holes. Both displays use the same colors and
graph topology for direct comparison. The old `Joint graph - filtered` display
and `/planning/filtered_joint_graph` publisher are removed.

New topics:

- `/planning/workspace_graph_mask`: MarkerArray, reliable/transient local,
  depth one. Original green nodes/orange edges remain visible with missing inputs.
- `/planning/workspace_graph_mask_status`: String JSON, same durability,
  status, hidden counts and calculation time. Changes only when markers change.

```bash
ros2 topic echo /planning/workspace_graph_mask_status \
  --qos-durability transient_local --once
```

The optional ROS parameter `publish_workspace_graph_mask` defaults true.
The old `publish_filtered_joint_graph` parameter no longer starts anything.
Original GNG marker topics remain available. Whole-arm offline methods such
as `PathPlanner.get_filtered_graph()` remain available but are not called by
the display. No graph-filter worker is created, even when enabled.

## Verification

`test_workspace_graph_mask.py` checks capsule/box intersections, exterior
endpoints, rotated boxes, degenerate/parallel segments, restoration, original
data identity, no subscriber work, no unchanged-scene publications, unknown/
expiry handling and renderer failure isolation.

`verify_workspace_graph_mask_ros.py` uses real PlanningNode DDS callbacks on
isolated domain 227 with the full stored GNG, synthetic human capsules and a
moving simulation box. The ordinary Planning and RRT timers are cancelled.
It checks visible startup without a camera, box masking while human data is
UNKNOWN, masking/restoration, no repeated stable-scene calculations/markers,
fresh EMPTY, human expiry, no graph-filter processes and zero Motion commands. Collision-query guards cover each display
call. This is not a moving-robot experiment.

```bash
ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 \
PYTHONPATH=src/planning:src/simulation:src/planning/test \
  python3 src/planning/test/verify_workspace_graph_mask_ros.py \
  --output /tmp/cra_workspace_graph_mask_ros.json

PYTHONPATH=src/planning:src/simulation \
  python3 src/planning/test/benchmark_workspace_graph_mask.py \
  --output /tmp/cra_workspace_graph_mask_benchmark.json
```

Recorded offline medians on this machine: about 0.54 ms for one synthetic
capsule, 5.08 ms for 16 capsules, and 5.49 ms for 16 capsules plus a box.
These measure XYZ mask calculations on 1000/4453 stored elements, excluding
ROS serialization, RViz rendering and existing Planning work. They are not
whole-cycle real-time guarantees. Current results, hashes and regression
counts for the original implementation are in
`results/workspace_graph_mask_20261119.json`. The no-camera correction and
YAML defaults are recorded in `results/camera_absent_mask_20261119.json`.

Interactive RViz, full live Kinect noise, and moving-robot avoidance remain
unverified for this change. Historical filtered-graph artifacts describe the
removed display and are retained for development history only.
