# Primary planner modes for CRA comparisons

Select the online primary path-search policy in `src/cra/config/robot.yaml`:

```yaml
planning:
  planner_mode: combined # gng / rrt / combined
```

Restart the normal CRA launch after editing. No additional launch argument or
workspace path is required. `combined` preserves the previous graph-first
behavior. Invalid values fail at startup.

| Mode | Primary search | Typical trade-off |
| --- | --- | --- |
| `gng` | Existing joint roadmap plus A*, including its existing local connectors and alternative goal candidates. RRT execution is disabled and its worker is not created. | Reuses the stored graph; scene-valid connectivity and goal coverage can limit success. |
| `rrt` | Select a shared valid goal, then request RRT-Connect directly. No primary A* or graph-alternative path search is run. | Searches outside stored graph edges; sampling can time out or produce longer routes. Worker startup and fresh-scene certification add latency. |
| `combined` | Try GNG/A* first. Only a completed NO_GRAPH_PATH batch can request RRT fallback. | Avoids RRT when the graph works; a failed graph search adds work before the RRT attempt. |

The current RRT implementation remains simulation-only, so selecting `rrt` or
`combined` on a physical-robot profile is rejected. `rrt` does not mean "move
despite invalid start/goal, UNKNOWN humans or manual STOP". It uses the same
bounded worker, two-scene certification, ownership checks and final Motion
handshake as the existing RRT fallback.

## What is shared and what is compared

These modes compare the **primary online path-search policy**, not GNG training
or a completely independent end-to-end RRT stack. All modes retain the same
target window, joint limits, collision geometry/margins, target GUI, goal
selection/IK recovery and Motion controller. Goal selection and visualization
still load existing workspace/graph data even in `rrt` mode; RRT path search
does not use its edges. No mode regenerates the stored graph.

`planning.astar.mode` remains the blocking/incremental scheduling choice. In
`rrt` mode, incremental scheduling applies only to goal selection: its status
is GOAL_SEARCHING and its recorded task is GOAL_SELECTION. The RRT worker is
already asynchronous. It is not secretly running A* in that mode.

Human Retreat and controller/contact fault recovery retain their existing
separate roles. In particular, special controller fault recovery can use the
existing graph-based recovery machinery; those SAFE_RECOVERY/SAFE_POSITION
motions are not primary-planner trials. RRT-only primary search failure does
not invoke the graph-based safe-position fallback. Report recovery episodes
separately when comparing policies. An invalid primary RRT start is rejected;
it is not permission to grow an RRT tree out of contact.

The experimental `simulation_only_ignore_unknown=True` policy is preserved
for ordinary graph operation. RRT still requires fresh OBSERVED or certified
EMPTY input. Fair trials should begin with fresh feedback, obstacles and human
data for all modes; do not compare a graph startup UNKNOWN exemption against
an RRT request that is correctly waiting for input.

## Worker settings and old configurations

`planning.rrt` now contains budgets, visualization and retry settings; its
normal YAML mode switch is replaced by `planning.planner_mode`. There is no
need to set a second worker mode. RRT-only and combined use the same search
and validation budgets.

Older configurations without `planner_mode` still resolve legacy `rrt.mode`
or `rrt_shadow.enabled`. The explicit diagnostic node parameter `rrt_mode`
remains for old off/shadow/fallback probes. Such an override changes the
effective primary policy where necessary, and the effective mode is logged;
do not use it in ordinary three-mode comparisons.

## Reading the logs

Startup, Planning snapshots/profiles and RRT events include `planner_mode`.
`planner_source` identifies the route actually executed; goal-selection
`source` separately says GRAPH or IK.

- `planner_mode=gng`, `planner_source=GNG_ASTAR`: normal graph route.
- `planner_mode=rrt`, `rrt_trigger=PRIMARY_RRT`: deliberate direct RRT request.
  `last_graph_failure` remains empty; no graph failure is fabricated.
- `planner_mode=combined`, `rrt_trigger=NO_GRAPH_PATH`: graph-first fallback.
- Pending goal work, IK exhaustion or invalid/unknown input do not count as
  graph failures. An RRT search budget expiry is a failure of that RRT trial,
  not permission to execute an uncertified path.

SEARCH_SUBMITTED events also include the RRT seed (the attempt sequence).
The reused backend still logs `RRT fallback` and FALLBACK_VALIDATED even for
primary RRT; read planner_mode and trigger to distinguish its actual role.
RViz workspace-graph visibility is independent of the route policy, so visible
graph markers do not prove that A* ran.
Useful comparison measures include path discovery latency, goal-selection cost,
success rate, waypoint count, joint-space path length, STOP/recovery counts,
input age and system CPU/RAM. Worker search time, elapsed wall time and Motion
completion time are different measurements. `cycle_work_ms` is not total CPU
usage and excludes asynchronous worker work.

Use identical start/target/obstacle/human conditions and repeat RRT trials;
the baseline RRT-Connect planner does not promise shortest paths or guaranteed
success under a fixed budget. The latest-only logger is unchanged: preserve
each trial's output before starting another launch if results must be compared.

## Verification

The mode-routing tests reject invalid settings, prevent RRT in `gng`, prove
that `rrt` bypasses A* even when the graph would work, and check goal-only
incremental cancellation plus shared freshness/manual-stop/certification gates.

An isolated native Planning -> C++ Motion -> headless MuJoCo probe uses the
same connected two-node fixture and synthetic FAR-human input for all three
modes. `gng` and `combined` execute GNG_ASTAR without RRT search events; `rrt`
executes RRT_CONNECT with PRIMARY_RRT. All complete the first movement and stop
on synthetic input loss. A separate disconnected control makes `combined`
request RRT with NO_GRAPH_PATH.

Reports are `results/planner_mode_gng_20261124.json`,
`results/planner_mode_rrt_20261124.json`,
`results/planner_mode_combined_20261124.json`, and
`results/planner_mode_combined_gap_20261124.json`.
The probe accepts `--planner-mode gng|rrt|combined --connected-graph`; the
disconnected control omits `--connected-graph` and uses `combined`.

These controls verify routing and execution ownership, not which algorithm
performs best with Kinect, moving humans, narrow passages or the full roadmap.
The clear fixture allows a direct RRT connection and is not a sampling-quality
benchmark.
