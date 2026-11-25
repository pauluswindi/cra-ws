# A* execution modes

Choose the primary graph-search scheduling mode in `src/cra/config/robot.yaml`:

```yaml
planning:
  astar:
    mode: incremental # blocking / incremental
    slice_budget_ms: 10.0
    slice_period_ms: 10.0
    max_operations_per_slice: 2048
  visualization:
    workspace_graph_rate_hz: 2.0
```

Restart the normal CRA launch after editing YAML. No new launch argument or
dataset path is required. Missing settings retain `blocking`; unsupported modes,
non-finite budgets and invalid operation limits fail at startup. Missing mode
still uses `blocking`; the current operator-selected trial uses `incremental`.

| Mode | Behavior | Cost |
| --- | --- | --- |
| `blocking` | Complete the primary graph search in its current callback. | Existing collision context; long searches can delay input callbacks. |
| `incremental` | Retain the search queues and candidate progress between timer slices. | One extra native MuJoCo model/data context, search state, and timer overhead. Graph arrays and adjacency are shared; no additional worker process. |

The time budget is cooperative: a single collision query, sorting operation or
native call can overrun it. The operation limit also yields during inexpensive
work. It does not mean "A* failed after 10 ms" and is not an end-to-end latency
guarantee. Total search latency can increase with scheduling or scene restarts.

## Graph-first policy

The same goal selection, diagnostics and bounded IK recovery apply in both
modes. IK origin does not force RRT: see [GOAL_SELECTION.md](GOAL_SELECTION.md).
The separate `planning.planner_mode` switch selects gng/rrt/combined; this
document's graph-first policy describes `combined`. See
[PLANNER_MODES.md](PLANNER_MODES.md). RRT-only uses incremental slices for goal
selection only and never runs the primary graph-search batch.

- An existing valid GNG/A* route keeps its normal execution and validation path.
- Pending incremental work reports `ASTAR_SEARCHING`, never `NO_GRAPH_PATH`.
- The existing lazy, eager, local-connection and intermediate-node searches are
  retained, including the configured alternative joint-goal candidates.
- Only a completed `NO_GRAPH_PATH` batch with a matching current scene may
  request RRT. Exhausting one slice never requests RRT.
- RRT admission, fresh-scene validation, Motion acknowledgements, human margin
  retreat and final execution checks retain their existing roles.

## Snapshot ownership and cancellation

The incremental workspace copies mutable collision geometry and original human
timestamps. Live callbacks update the main checker while the search uses a
consistent private scene. New frames never extend that snapshot's deadline.
The workspace is reused across attempts; it does not reload XML or regenerate
the roadmap for every frame.

Target/obstacle invalidation, start movement beyond the existing 0.01 rad
stability threshold, stale feedback/obstacles, missing required human data,
manual STOP and Motion recovery/fault cancel pending work. The regular manager
still decides every STOP, RELEASE and EXECUTE; the slice timer publishes none.

A failed search against changed human geometry is discarded as
`ASTAR_REPLANNING`, rather than claiming the live graph has failed. Successful
paths keep the snapshot revision and must pass the existing live human guard
and execution preparation before dispatch. Snapshot expiry is also a retry,
never permission to start RRT blindly. Rapidly changing scenes may repeatedly
invalidate searches; this version does not certify old failure results for a
different human pose.

This setting covers the primary graph-search batch. Human Retreat, special
recovery and final execution validation retain their existing scheduling.
Selecting incremental mode does not impose a 10 ms bound on the
whole Planning callback or fix camera SDK timeouts and Motion `STALLED` faults.

## Scheduling tuning and diagnostics (2026-11-24)

The operation cap is now 2048 rather than 256. The 10 ms soft time budget and
10 ms timer period are unchanged. Expensive operations still yield at the time
budget; inexpensive graph traversal can use more of each slice before yielding.
No search is declared failed merely because a slice ends.

`Planning profile.astar_search` now reports `operations_total`,
`last_slice_operations`, `last_yield_reason` (`TIME_BUDGET`, `OPERATION_LIMIT`,
`COMPLETE` or `ERROR`), `time_budget_yields` and `operation_limit_yields`.
Operations count cooperative checkpoints, not expanded graph nodes or FK calls.
`search_failure` retains the original planner outcome. The separate `reason`
identifies `SNAPSHOT_EXPIRED`, `SCENE_CHANGED` or `SNAPSHOT_DATA_UNAVAILABLE`
when a failed attempt is retried. Expiry during an initial state check cannot
misclassify an old snapshot as a live `START_INVALID` recovery request.

Workspace graph display work is limited to `planning.visualization.workspace_graph_rate_hz`
(2 Hz by default). Each due call reads only the latest scene; intermediate
visual updates are not queued. No subscriber means no work and does not consume
the display slot. Collision validation and Motion dispatch are not throttled.
The display may lag by roughly one configured period plus executor delays;
one individual display calculation can still take longer than an A* slice.

## Verification

- 214 targeted tests pass, including graph-route equivalence, suspended expiry,
  graph-first RRT admission, cancellation and latest-scene collision rejection.
- `planning` and `cra` build successfully; changed Python files pass flake8.
- The isolated native probe accepts `--astar-mode incremental` and optionally
  `--astar-max-operations 4`. It writes a temporary YAML override, uses a
  disconnected two-node fixture and synthetic human messages, and runs native
  Planning -> C++ Motion -> headless MuJoCo. It checks that input callbacks can
  occur between slices, graph failure precedes RRT, the first RRT motion
  completes, and invalid human input stops subsequent RRT motion.
- Reports: `results/astar_incremental_simulation_20261124.json` and
  `results/astar_incremental_sliced_simulation_20261124.json`.
- The scheduling update was checked in
  `results/astar_scheduling_simulation_20261124.json` with the 2048-operation
  setting: native RRT motion completed and synthetic data loss stopped Motion.
- `benchmark_astar_scheduling.py --case replan` uses the actual roadmap/model,
  a recorded robot/target pair, FAR synthetic humans and a virtual timer that
  accounts for measured native-query overruns. This is a controlled workload,
  not a reconstruction of the moving-human scene. Both caps produced the same
  route and 16836 checkpoints. Slices fell from 79 to 24 and operation-cap
  yields from 61 to 3; measured work was about 332/334 ms. Modeled elapsed time
  was about 888/357 ms without competing callbacks. See
  `results/astar_scheduling_ab_20261124.json`; these are not live latency claims.

These controlled fixtures do not establish live Kinect performance, behavior
under moving humans, or improved FPS on the full workspace graph.
