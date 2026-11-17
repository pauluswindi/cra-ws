# Two-second human timeout and exact geometry memoization

The user approved a 2 s simulation trial together with collision-checking
optimization. `planning.human_avoidance.data_timeout` in CRA robot.yaml is now
2.0 s. PlanningNode applies that value to both HumanCapsuleMonitor and
CollisionFilter. The existing capture/receipt deadlines, invalid-input handling,
human/static margins, GNG/A*, Motion guards and execution flags remain in place.
This timeout change does not repair upstream `valid=False` frames or establish
that a route exists when a local retreat search exhausts its iteration budget.

## Algorithm and ownership

The new memoization lives in CollisionFilter's human-distance query, below the
existing A* node/edge caches. It stores at most 256 geometry reports using LRU
eviction (discarding the least recently reused report when full).

The key contains the environment revision, immutable capsule values, margin,
all-pairs/report mode, frame conversion and exact raw FK geometry inputs:
base pose, robot geometry positions/rotations/radii, and mesh-box centers/half
extents with their array shapes. Model geom names are constant for a loaded
checker. There is no angle rounding or approximate distance lookup. Preparing
the same frame transforms and calculating sphere/mesh-box distances is skipped
on a hit; every caller still performs its original FK/static-contact checks.

The cache is cleared when a human snapshot is applied, including a fresh frame
with identical stationary geometry. Obstacle updates and changes to human-check
enablement also clear it. Receipt/capture freshness and environment revision are
checked before/after geometry use; looking up a result never refreshes the human
timestamp. UNKNOWN results are not stored. Diagnostic dictionaries and retreat
pair dictionaries are copied so callers cannot modify stored certificates.
All-pair retreat reports use a separate key from ordinary nearest-pair reports.

The runtime setting is:

```yaml
planning:
  collision_checks:
    human_report_cache_size: 256
```

Use zero to disable this memoization. Size must be a non-negative integer.
`Planning profile` logs include `human_geometry_cache` capacity, current entries,
`hits_total` and `misses_total`. Totals are cumulative per checker; entries reset
at snapshot boundaries. A miss means a full human geometry calculation, not
necessarily an unsafe configuration. These counters are diagnostic only.

## Verification

149 Planning tests pass, including 13 new cache tests for freshness, capture
expiry, expiry during queries, identical/new snapshots, moving humans, static
contacts, mutation isolation, all-pair reports, margin/mesh changes, bounded
storage and random full-geometry equivalence. Changed Python passes Flake8.
CRA/Planning builds pass. Installed source hashes and native ROS construction
on isolated localhost domain 223 verify timeout 2.0 s at both layers, cache 256,
0.10/0.005 m margins, execute_path=True and simulation_only_ignore_unknown=True.
The constructor test does not spin callbacks or issue a motion command.

Reproduce the controlled geometry comparison:

```bash
PYTHONPATH=src/planning:src/simulation:src/planning/test:$PYTHONPATH python3 \
  src/planning/test/benchmark_collision_memoization.py --repeats 7 \
  --output /tmp/cra_collision_memoization.json
```

Each variant starts with a cold planner/graph cache. Snapshot time is frozen to
compare identical geometric work, not to simulate freshness behavior. Runs are
interleaved using a fixed shuffle. Every returned path is independently checked
again with memoization disabled, and exact path hashes/failure results match.

| Fixed scene | Full geometry median | Memoized median | Geometry calculations |
| --- | --- | --- | --- |
| Detour, one capsule | 102.1 ms | 82.5 ms | 517 to 241 |
| Detour, eight capsules | 108.2 ms | 85.9 ms | 517 to 241 |
| Margin retreat | 139.1 ms | 125.3 ms | 424 to 319 |

The eight-capsule scene adds seven far capsules to the existing synthetic detour;
it is not a replay of a live full-body human. These seven-repetition medians show
19-21% lower planning time and approximately 10% lower retreat time for the
tested work. They do not establish a universal speedup or success in live ROS.

Separately, compare actual elapsed freshness budgets on stationary geometry:

```bash
PYTHONPATH=src/planning:src/simulation:src/planning/test:$PYTHONPATH python3 \
  src/planning/test/compare_human_timeouts.py --repeats 2 \
  > /tmp/cra_timeout_2s_comparison.json
```

Both repeated trials accept a 1.5 s-old input and find a path under the 2 s
budget; the 1 s budget rejects that input. Inputs already 2.1 s old are rejected
under both budgets. These results validate the intended deadline difference,
not moving-human safety or the complete ROS/Motion pipeline.

Artifacts are test/results/collision_memoization_20261117.json and
test/results/human_timeout_2s_memoized_20261117.json, with source hashes and
per-trial results. The benchmark keeps full collision samples and geometry.

## Next live trial

Restart the previous CRA launch to load the newly built Planning source. Startup
must show `human_data_timeout=2.000s, human_geometry_cache=256`. Preserve the usual
scene and targets, capture the full log, and compare path_search time, freshness
failures, STOP reasons and cache counters. Initial comparison with a stationary
human separates computational expiry from scene changes. Native ROS/viewer/Motion
execution with this combined change remains unverified. The separate simulation
clock scheduling proposal and global retreat search design are unchanged.
