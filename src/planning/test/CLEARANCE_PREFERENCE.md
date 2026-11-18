# Offline human-clearance preference

The user approved preferring extra human clearance even at modest extra joint
travel. This document records the offline experiment without live behavior
changes. Its subsequent worker observation integration is in `RRT_SHADOW.md`;
the selector still cannot supply Motion commands.

## Mechanism

`ClearancePathPreference.improve(baseline, seed)` operates on an existing valid
N-by-5 joint path. It scores the minimum clearance along each segment (not the
average or only waypoint clearance). It tries one RRT-Connect candidate with a
preferred 0.07 m clearance. Fixed start/goal clearances cap the effective target;
preference cannot demand more than the unchanged endpoints can provide.

The original mandatory human margin remains 0.05 m. Candidate search uses an
isolated copy of MuJoCo data and geometry cache, sharing only immutable model
geometry. Mandatory clearance is not overwritten. Search guidance can reject
a point for insufficient preferred clearance, while final acceptance always
uses the original checker. Candidate endpoints must exactly match the baseline.
Candidate joint path length must be at most 1.25 times the baseline length and
its bottleneck clearance must improve. Otherwise the baseline is retained and
recertified. Failure to find the preference is not failure to find any route.

The selector has a 2 s cooperative budget, including scoring, one candidate
search capped at 0.8 s and final recertification. Half of the remaining time is
reserved when assigning the search budget. Baseline RRT generation is outside
this budget and remains capped at 0.5 s. Expiry, scene changes, final validation
failure, query exceptions or total deadline overrun yield no result; a stale
baseline certificate is never returned as a fallback. These are offline budgets,
not acceptable live feedback latencies. The selector is not wired into launch.

## Consistent geometry is necessary for ranking

CRA's ordinary boolean check can stop at a conservative bounding-sphere
clearance once the margin is certified. It refines mesh boxes when proximity
requires it. Fast FAR reports can therefore provide different-quality lower
bounds at different configurations. Ranking those values directly can create
an artificial preference barrier or suggest spurious improvement when a
threshold changes the refinement branch.

Preference scores instead always request the existing all-pair refinement
(`include_pairs=True`) at the same 0.05 m mandatory threshold. Both baseline
and candidate use this consistent sphere/mesh-box metric. The search clone
uses the same refinement and applies preference after the distance calculation.
The immutable shapes, radii and collision rules are unchanged. Native tests
verify independent mutable state/cache and inability to borrow a newer snapshot.
Distances remain conservative model bounds, not physical mesh surface accuracy.

## Reproduction

Inside cra-humble, from the workspace:

```bash
PYTHONPATH=src/planning:src/simulation:src/planning/test python3 \
  -m unittest test_clearance_preference -v
PYTHONPATH=src/planning:src/simulation:src/planning/test python3 \
  src/planning/test/benchmark_clearance_preference.py --seeds 8 \
  --output /tmp/cra_clearance_preference.json
```

The comparison pairs baseline and selected paths for the same eight seeds on
the existing synthetic capsule blocking the direct segment. Every returned
path is prepared with the existing Motion segment limit and independently
validated using fresh MuJoCo data, no geometry cache and 0.01 rad edge samples.
Both the fast lower-bound metric and consistently refined metric are recorded.
Then the capsule is shifted by +/-1, +/-3 and +/-5 mm separately along each
base_link axis: 18 shifted scenes plus the nominal scene per path. The same
unchanged path is checked against each scene without repair or replanning.
This is a deterministic geometric perturbation test, not a sensor-noise model,
filter, future-motion predictor or complete continuous-time safety proof.

Human monotonic time is deliberately frozen; CPU budgets use real perf_counter.
Preference performs extra work, so its cost is reported separately rather than
claimed to beat baseline computation time. No real human, camera, ROS graph or
robot is involved. Best-first retreat and completed-task hold policy remain
separate work. Evidence: results/clearance_preference_20261118.json.

## Verified experiment, 2026-11-18

Eight paired seeds each returned a baseline and selected path. All sixteen
nominal paths passed independent certification after preparation for Motion.
All eight selected alternatives improved refined bottleneck clearance while
respecting the 25% joint travel cap; the greatest individual increase was
20.46%. Some alternatives were shorter too, which is possible with random
sampling and does not establish optimality.

| Metric | Baseline | Preferred selection |
| --- | ---: | ---: |
| Median refined minimum clearance | 0.051373 m | 0.070838 m |
| Median joint path length | 4.8412 rad | 5.0923 rad |
| Nominal scene validity | 8/8 | 8/8 |
| +/-1 mm axis shifts | 44/48 | 48/48 |
| +/-3 mm axis shifts | 34/48 | 48/48 |
| +/-5 mm axis shifts | 32/48 | 48/48 |

Each shifted row contains six axis/sign offsets for each of eight paths, not
48 independent random human trajectories. Selection adds median 1.354 s
(range 0.909-1.710 s) beyond external baseline search. Human time is frozen;
these measurements do not show that a live 2 s observation stays valid during
the whole planning/feedback/dispatch pipeline.

All 182 Planning functional tests pass, including 17 preference tests and 16
earlier RRT tests. The eight-seed benchmark also exercises rejection at an
intermediate edge sample: its diagnostic recorder handles both waypoint and
nested edge-state failures. Recorded source hashes match the experiment files.
Planning builds with colcon; new Python files pass Flake8 at 99 columns, and
installed module imports pass with Planning and Simulation package environments.
Live integration remains pending a shared timing/freshness design; keep GNG/A*
as the primary route and discuss scheduling before adding fallback dispatch.
