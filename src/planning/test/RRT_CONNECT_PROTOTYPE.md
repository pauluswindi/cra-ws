# Bounded RRT-Connect prototype

Date: 2026-11-18. This records the offline first step, not a live fallback.
The existing PlanningManager, goal selection, A*, Motion, launch configuration,
5 cm human margin, 2 s human timeout and simulation flags are unchanged.
The existing completed-task/retreat behavior is not changed by this prototype.
The later observational worker integration is documented in `RRT_SHADOW.md`;
RRT still does not supply Motion commands.

## Algorithm and scope

`planning.algorithms.rrt_connect.RRTConnectPlanner` builds two temporary trees
in finite five-joint configuration space. It first checks a direct segment;
otherwise it samples joint configurations uniformly, extends the nearer tree
by a bounded Euclidean joint step, and repeatedly extends the other tree to
connect. Tree roles alternate. Parent chains recover a start-to-goal path;
the assembled result is checked again before returning. The default step is
0.15 rad, 1000 outer iterations, 3000 total nodes and 0.5 s of cooperative wall
time including final certification. Fixed seeds reproduce sampling, not timing.
The node cap also bounds the inner CONNECT loop.

Every new configuration and edge uses CRA's existing `CollisionFilter`.
Environment revision and required human freshness are checked before and
after checker calls. Reversed goal-tree edges are certified in execution order.
No graph mutation, persistent tree reuse, margin recovery or motion publication
is performed. This is RRT-Connect, not the reusable R-RRT* from the palletizing
paper. A* remains the live algorithm. Initially there were no runtime callers;
the later worker observes candidates without dispatching them.

API: `path = RRTConnectPlanner(checker).plan(start, goal, seed=0)`. A path is
N-by-5 joint angles; identical valid endpoints return one stationary point,
which must not be sent as a movement command. A direct path can have a large
joint delta, so an eventual integration must use `prepare_execution_path`
and Motion's existing segment limits. A failed request returns None and an
explicit `last_plan_failure`, including START_INVALID, GOAL_INVALID,
HUMAN_VISION_UNKNOWN, ENVIRONMENT_CHANGED, TIME_BUDGET_EXHAUSTED, ITERATION_LIMIT,
NODE_LIMIT, PATH_INVALID or COLLISION_CHECK_ERROR. Search limits do not prove
geometric impossibility. `last_statistics` records timing, nodes, iterations,
query counts and returned path length; query exceptions retain `last_error`.

This is a geometric algorithm with sampled collision checks. It does not
guarantee continuous collision freedom, optimality, physical trackability or
safe behavior against future human motion. A single expensive checker call
cannot be interrupted: time limits reject late results, rather than providing
a hard real-time execution bound. Do not share one mutable checker between
concurrent planners. Live integration must address fresh feedback, target
cancellation, total planning budgets and the STOP/RELEASE/EXECUTE protocol.

## Reproduce without camera, ROS nodes or robot commands

Inside cra-humble, from the workspace:

```bash
PYTHONPATH=src/planning:src/planning/test python3 -m unittest test_rrt_connect -v
PYTHONPATH=src/planning:src/simulation:src/planning/test python3 \
  src/planning/test/benchmark_rrt_connect.py --seeds 8 --rrt-budget 0.5 \
  --output /tmp/cra_rrt_connect_offline.json
```

The 16 unit tests use independently calculated exact rectangular-wall
intersections embedded in five dimensions, not a duplicate RRT implementation.
They cover detour recovery, certified branch assembly, deterministic seeds,
invalid inputs/endpoints, finite budgets and expiry/revision/exception failure.

The real-model experiment compares the existing PathPlanner with RRT-Connect
on the same explicit start and joint goal, using eight seeds/repeats per method
in five scenes (80 trials):

- A far human: normal free-space reference.
- One synthetic human capsule on the midpoint link4 position: both endpoints
  are valid but the direct joint segment is blocked.
- The same blocked scene with an artificial two-node, zero-edge graph: proves
  whether new configurations can bridge absent graph connectivity. This is a
  deliberate control, not a diagnosis of the full live GNG graph. Success
  requires actual tree growth and more than two returned waypoints.
- A non-contact start at 3 cm human clearance: both ordinary planners must
  reject it under the current 5 cm margin, without an escape exception.
- An expired observed human: neither planner may return a path.

Observations are deliberately frozen with a monotonic clock for geometry
comparison; the RRT perf_counter deadline still advances in real wall time.
This experiment does not establish moving-human performance. Each method gets
a fresh model/checker and cold caches. Existing A* limits are preserved without
introducing a wall deadline, so the timings are not an equal-budget race.
Every returned path is prepared with the existing segment limit and recertified
using new MuJoCo data, zero geometry cache and 0.01 rad edge sampling (twice as
fine as the usual checker). Minimum sampled clearance and source hashes are
saved in `results/rrt_connect_offline_20261118.json` along with prepared paths.
The experiment certifies paths against the configured geometric approximations,
not actual body surfaces or live servo dynamics.

## Verified first experiment

All 165 Planning functional tests pass (149 baseline and 16 new RRT tests).
Planning builds successfully and new Python files pass Flake8 at 99 columns.
The final saved experiment contains 80 trials and 40 independently recertified
paths; source hashes match the benchmark and planner files.

| Scene | Existing A* successes | RRT-Connect successes |
| --- | ---: | ---: |
| Far human | 8/8 | 8/8 |
| Human blocks direct segment | 8/8 | 8/8 |
| Artificial graph gap plus blocked direct segment | 0/8 | 8/8 |
| Start inside margin at 3 cm | 0/8, START_INVALID | 0/8, START_INVALID |
| Expired observation | 0/8, HUMAN_VISION_UNKNOWN | 0/8, HUMAN_VISION_UNKNOWN |

On the blocked-direct scene, median search time is 0.315 s for existing A* and
0.234 s for RRT-Connect. Median joint path lengths are 7.011 and 4.841 rad.
These are scene-specific observations with different existing search limits,
not evidence of universal speed/length superiority. Prepared paths have the
same start/goal and current motion segment limit. The smallest independently
sampled fast RRT clearance lower bound is 0.050079 m, only about 0.079 mm above
the configured human margin. This metric can be an unrefined sphere bound;
the later preference experiment uses consistent box-refined scores instead.
Frozen-scene success alone does not prove
robustness to noisy/moving geometry. That first experiment included no clearance
preference, hysteresis, retreat extension or live dispatch. The subsequent
user-approved offline preference experiment is documented separately in
`CLEARANCE_PREFERENCE.md`; live dispatch remains unchanged.

## References and next decisions

- Kuffner and LaValle (2000), RRT-Connect: An Efficient Approach to Single-Query
  Path Planning: https://www.clear.rice.edu/comp450/papers/kuffner_lavalle_00.pdf
- Current GNG graph and collision/path checks remain the primary CRA baseline.

Before live integration, discuss bounded fallback dispatch only after a true
NO_GRAPH_PATH with valid start/goal and fresh observations; limits/unknown data
must stay distinguishable. Also implement the separately discussed completed
task hold policy. Best-first retreat and reusable path repair remain later
development steps; neither is implemented by this prototype.
