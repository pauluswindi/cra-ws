# Retreat search and unchanged camera settings

Implemented after the 2026-11-19 runtime log; verified on 2026-11-20.

## Findings from the supplied log

RRT is configured as `fallback`, but every one of the 85 emitted Planning
profiles says `rrt_result=NOT_TRIED`. Six A* searches report PATH_FOUND; there
is no NO_GRAPH_PATH, RRT_SEARCHING or source=RRT_CONNECT. This is consistent
with RRT's admission rule, not evidence of an unloaded RRT implementation.

The long stop is HUMAN_RETREAT_BLOCKED. Reported current clearance includes
0.01812 m and 0.00747 m, below the unchanged 0.05 m margin. Ordinary RRT cannot
use that invalid start. The existing retreat uses a separate certificate:
no static contact, no human overlap, no worsening of any initially close
robot/human pair, and no entry into a previously clear pair's buffer.

`SEARCH_BUDGET_EXHAUSTED` in this retreat routine means its 32-step limit,
not a wall-clock timeout. Emitted `retreat_search` stages average 273.0 ms
and reach 439.786 ms in the supplied excerpt. They are synchronous. The
log lacks the full capsule geometry, so it cannot establish why the search
failed for that exact scene or whether an admissible retreat existed.

Camera capture fails twice after five consecutive timeouts; the first miss
precedes the heavy retreat interval. Color queue-drop messages begin after
the first reported timeout. This does not establish whether depth capture,
synchronization, USB transport or system load caused the original problem.
Kernel messages in the matching interval show USB interface-claim warnings,
but do not identify a demonstrated transport failure.

## Small correction and computational cost

`PathPlanner.find_human_margin_retreat()` still tries +/-0.01 rad changes
to each of five joints, with the same 32-step limit and joint/segment bounds.
It now sorts improving endpoint candidates by gain before checking complete
transitions. Once the highest-gain valid transition passes, lower candidates
need no transition checks. Stable ties retain joint/sign order. Candidate
endpoints never authorize Motion on their own.

A reproducible defect also exists at local dead ends: the old branch returns
None immediately even after progress satisfying the existing staged-retreat
minimum. The step-limit branch already allowed that progress. Both exits now
consider it under the SAME minimum gain (obstacle safety margin, 0.005 m) and
full start-to-end monotonic certificate. Manager preparation, RELEASE and
final execution validation remain mandatory and unchanged.

For example, a deterministic control improves from 0.018 m to 0.024 m and
then reaches a plateau. The old search discards it. The corrected search may
return that certified 6 mm improvement, remain in the human margin, stop,
and consider another stage using fresh data. Improvements below 5 mm,
invalid full transitions, expired data and changed scenes still return None.
This control reproduces the defect; it is not a replay of the user's scene.

This remains a greedy coordinate search: it cannot discover every coordinated
multi-joint escape or certify that no solution exists. It is also still
synchronous and has no new hard wall-clock guarantee. Do not increase budgets,
relax pair constraints, reduce margins or force RRT on invalid starts to hide
these limits. Broader or asynchronous retreat search needs a separate design.

## Diagnostics

Planning profiles now include `retreat_search` only when that cycle invokes
the search. It contains step limit/completed steps, screened candidates,
endpoint checks, full transition checks, initial/final clearance, gain,
required gain, final transition verdict, stop condition and elapsed time.
Endpoint counts exclude checks inside transition certification; the benchmark
separately counts all state calls. No search/visual worker or marker stream
is added. Full capsule dumps are not emitted.

Stop conditions distinguish STEP_LIMIT, NO_MONOTONIC_STEP and MARGIN_CLEARED.
The existing reason SEARCH_BUDGET_EXHAUSTED is retained for compatibility.
A missing final-transition verdict means the search never attempted that
certificate, for example because gain was below the required threshold.

## Verification and reproduction

Final results: 296 Planning tests pass in 29.836 s, including 10 new retreat
cases; Planning builds in 0.98 s, Flake8 and whitespace checks pass. Twelve
native scenes retain all 10 previously successful paths and reduce complete
transition calls from 346 to 228; two scenes remain blocked. The installed
RRT/C++ Motion/MuJoCo integration again reaches its goal, then stops when
synthetic human data is invalidated. This does not exercise live-human retreat.
The final 60 s real-camera probe has zero capture timeout/color queue drop,
1375 received skeleton messages and complete driver cleanup at unchanged
settings. These scopes and all source hashes are recorded in the result file.

`test_human_retreat_search.py` covers the plateau, insufficient gain, invalid
straight certificate, expiry/revision changes, top-candidate rejection,
deterministic ties and worsening another close pair. Existing native geometry
and manager tests continue to enforce the actual clearance and Motion gates.

After sourcing the ROS environment, from the workspace:

```bash
PYTHONPATH=src/planning:src/simulation:src/planning/test \
  python3 -m unittest test_human_retreat_search test_dynamic_replanning

PYTHONPATH=src/planning:src/simulation:src/planning/test \
  python3 src/planning/test/benchmark_human_retreat.py \
  --output /tmp/cra_retreat_benchmark.json
```

The benchmark extracts the reference method from Git HEAD by default; use
`--reference-revision <commit>` to preserve the reference after committing.
It reuses one native model, freezes human observation time for equal geometry,
and uses real perf_counter timing. Twelve synthetic capsule scenes include
one logged joint pose, not the missing logged capsule geometry. Existing
successful paths must remain identical and returned segments are recertified.
These are offline checks, not live stop-latency or dynamic-human guarantees.

## Camera diagnostic without changing settings

The user explicitly retained 30 FPS, RGB 1536P, smoothing and calibration.
No camera launch settings, driver code, retry count or Planning freshness
limit were changed by this correction.

`src/azure_kinect_ros_driver/test/probe_capture_stream.py` reads camera settings
verbatim from the existing launch, refuses to run beside another Kinect driver,
and requires domain 229. It runs only the installed camera driver plus stream
subscriptions; no Planning, Motion, reconstruction or viewer starts, and no
image payloads are saved. Only this probe's own child process is stopped.
Use it after closing the ordinary CRA launch:

```bash
ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1 \
  python3 src/azure_kinect_ros_driver/test/probe_capture_stream.py \
  --duration 60 --output /tmp/cra_capture_probe.json
```

It records received-message rates, gaps, capture ages, driver exit/cleanup,
timeout/queue-drop evidence and exact parameter values. Received rates can
include DDS/subscriber losses and must not be called the sensor's FPS.
A successful isolated run does not reproduce full CRA load or prove a repair
of intermittent camera failures. See `results/retreat_debug_20261120.json`
for measured results and remaining limits.
