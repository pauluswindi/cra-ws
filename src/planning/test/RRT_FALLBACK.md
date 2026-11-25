# RRT-Connect fallback in MuJoCo

The default `planning.planner_mode: combined` retains the fallback behavior
below. `gng` disables RRT execution and `rrt` selects direct primary RRT search
with the same worker/certification gates. See [PLANNER_MODES.md](PLANNER_MODES.md).

GNG+A* remains the primary planner. After ordinary graph search and its
configured alternative joint goals fail with `NO_GRAPH_PATH`, one bounded
RRT-Connect attempt may supply the next path. `planning.planner_mode: combined`
enables this for simulation only. It is not RRT-only planning, a replacement
for recovery from an invalid start, or evidence of tested live-human avoidance.

## Algorithm and execution ownership

Goal selection can additionally supply a collision-validated IK endpoint when
retained/graph goals are unavailable. That endpoint still goes through A* first;
only NO_GRAPH_PATH admits RRT. See [GOAL_SELECTION.md](GOAL_SELECTION.md).

GNG+A* searches stored joint configurations and connections. A disconnected
roadmap may fail even when free joint-space motion exists. RRT-Connect can
sample new five-joint configurations, grow its existing start/goal trees and
connect them using the same CollisionFilter state/edge checks.

The integration uses the existing worker, not another collision model or
parallel search backlog:

1. PlanningManager exhausts its usual graph/alternative-goal attempts. With
   valid input, fresh feedback/obstacles, fresh OBSERVED or EMPTY human data,
   an idle/stopped robot and no active recovery/fault, it enters RRT_SEARCHING.
   This precedes the existing safe-position recovery. A STOPPED robot must
   already be owned by Planning; a manual stop is not released by this feature.
2. `RRTFallbackCoordinator` submits the existing `run_job()` in the spawned
   worker. Fallback uses baseline RRT without the extra clearance-selection
   phase. Mandatory collision margins remain unchanged. Shadow mode retains
   its optional clearance preference experiment.
3. Search output is only a candidate. A second worker job takes a NEW scene
   snapshot and calls the actual `PathPlanner.prepare_execution_path()`,
   including its densification, segment/waypoint limits and full certification.
4. A prepared result carries attempt/target identifiers, start configuration,
   scene revision and an expiry capped by the original request deadline.
   Target/attempt changes, active Motion/recovery, stale inputs, moved start,
   expired deadlines and malformed paths reject the result. A scene change
   preserves the prepared candidate with its ORIGINAL revision, not a current
   safety certificate. New observations never extend an admitted request's deadline.
5. The node delivers a mailbox result only. The regular PlanningManager step
   adopts it as RRT_PATH_READY, source RRT_CONNECT. It then follows the existing
   execution preparation, final collision validation and feedback gates. A
   stopped robot follows RELEASE -> acknowledgement -> validation -> EXECUTE.
   Neither the coordinator nor a marker callback publishes Motion commands.
6. On failed/timed-out fallback, existing safe-position recovery may be tried.
   Changed-context results request primary replanning. Running discarded jobs
   drain before another submission; a late result cannot revive an old attempt.

During an active RRT trajectory, UNKNOWN/expired human observations request
STOP even if the primary simulation experiment can otherwise ignore UNKNOWN.
Fresh EMPTY observations permit static-obstacle planning only until their
original expiry. Human collision and path revalidation during execution still
use the existing manager. Invalid starts, human-margin intrusions, unreachable
goals and Motion faults retain their existing handling; they do not authorize
ordinary RRT to escape a collision.

## Changing human geometry during RRT handoff

Both primary `rrt` and the RRT phase of `combined` retain a successful worker
candidate when the scene changes during validation or mailbox delivery.
`CANDIDATE_REVALIDATION_REQUIRED` identifies a worker result whose revision
already differs from the live scene. The manager compares revisions again
at adoption, so changes after delivery receive the same treatment.

The serialized manager callback calls the real `prepare_execution_path()` on
the latest live scene before adopting such a candidate. It temporarily retains
RRT's finer edge sampling (`min(0.01, original max_joint_step)`) and restores
the checker afterward. It rejects collisions, stale feedback/obstacles/human
data, target/start/context changes, a further revision change and expired
deadlines. Only successful current-scene certification produces RRT_PATH_READY;
the normal preparation, final validation and STOP/RELEASE gates still follow.
This does not change the GNG/A* search or its adoption policy.

The existing `validation_budget` also caps manager handoff validation within
the ORIGINAL admitted request/certificate deadline. This is a soft limit:
a native collision call cannot be interrupted, but an overrun cannot be adopted.
Profiles include `rrt_handoff_validation` (reason, old/new revisions, elapsed_ms).
No persistent path pool, extra worker or repeated worker validation loop is added.

During retry cooldown, retain ONE pending request with RRT_WAITING_RETRY /
WAITING_RETRY instead of producing RETRY_COOLDOWN failures or new attempts.
Waiting ends no later than `retry_not_before + fallback_budget`; a missed
deadline rejects it. Admission uses a fresh scene and checks target, start,
fresh inputs and Motion ownership. The search allowance starts on actual
admission, once; subsequent human observations cannot extend it. Human Retreat,
manual STOP, invalid data and active recovery retain priority.

`verify_rrt_fallback_simulation.py --changing-human` changes a distant synthetic
capsule at 20 Hz while using real Planning, C++ Motion and headless MuJoCo.
It requires successful changed-revision handoff, completed motion and STOP
on invalid human input. Markers can remain as explicitly labeled historical search data; they never
certify execution. See [RRT_VISUAL_MODES.md](RRT_VISUAL_MODES.md). This is not a live Kinect,
near-human avoidance, or worst-case latency test.

## Input refresh after a slow failed primary search

A graph/alternative-goal batch can outlast the 1 s feedback/obstacle freshness
window while the single-threaded Planning callback is occupied. After a known
`NO_GRAPH_PATH`, stale feedback or obstacles now retain one lightweight
`rrt_refresh_attempt` rather than immediately repeating the primary batch.
The manager returns `RRT_WAITING_FOR_FEEDBACK` or `RRT_WAITING_FOR_OBSTACLES`
so queued callbacks can run. No worker job or Motion command is issued yet.

The retained attempt contains its sequence, target generation, start, goal and
one fixed deadline, created at primary failure using the existing fallback
budget. Waiting consumes that allowance; new observations cannot extend it.
On fresh input, current start and saved goal are collision-checked against the
latest human scene. The request then uses the existing worker, fresh-scene
validation and normal execution gates, with the original deadline retained.
No old path certificate is reused. Human scene updates can be considered for
this new search only after endpoint revalidation; static obstacle changes
invalidate the saved attempt through the existing obstacle-update path.

Target changes, start motion beyond the existing 0.01 rad tolerance, faults,
external STOP, active recovery, invalid/expired human data or an elapsed
deadline cancel the retained attempt. Human proximity handling still runs
before refresh admission: HUMAN_TOO_CLOSE cancels it and gives the existing
Human Retreat logic priority. Other planning/recovery interfaces are unchanged.

Profiles now include `rrt_admission_reason` and `rrt_refresh_pending`. Reasons
include FEEDBACK_STALE, OBSTACLES_STALE, HUMAN_TOO_CLOSE, EXTERNAL_STOP and
RECOVERY_ACTIVE. `rrt_result` records cancellation reasons such as
CANCELLED_TARGET_CHANGED, CANCELLED_START_MOVED and CANCELLED_HANDOFF_EXPIRED.
RRT_SEARCHING means an actual fallback request was admitted, not merely a
saved graph failure. No feedback timeout, human timeout or margin is increased.

This change improves handoff after a completed failed graph search. A* itself
still executes synchronously; it is not a hard response-time bound and does
not eliminate waiting when the current robot pose is unsafe or no route exists.

## YAML and running

Current `src/cra/config/robot.yaml`:

```yaml
planning:
  rrt:
    mode: fallback              # off / shadow / fallback
    visualization_enabled: true
    fallback_budget: 1.5         # seconds, including worker startup and handoff
    search_budget: 0.5
    validation_budget: 0.5
    retry_interval: 3.0
```

The configured total allowance is also capped by remaining source human-data
freshness. Cooperative budgets cannot interrupt a single native collision
query; late output is rejected. Existing A*, clearance monitoring and final
execution checks still run synchronously, so this is not a whole-node hard
real-time guarantee. Geometry can change faster than certification completes;
rejection in that case is expected, not a reason to soften margins or extend
data validity.

`mode: shadow` preserves observational search and its clearance selection;
`mode: off` creates no RRT worker. The old `planning.rrt_shadow` YAML remains
supported when the new block is absent. Active fallback is rejected before
node/model creation if `robot.mode` is not `simulation`. Mode changes require
restart. A ROS `rrt_mode` parameter override exists for isolated test probes.

After rebuilding planning/cra and restarting the old launch:

```bash
ros2 launch cra cra.launch.py rviz:=true
```

Dataset defaults come from the existing YAML fields. To inspect admission,
validation or rejection from another terminal in the same domain:

```bash
ros2 topic echo /planning/rrt/events
```

The legacy `/planning/rrt_shadow` String also receives events, now explicitly
tagged with mode and observation_only. Console output says `RRT fallback:`.
Useful signs are SEARCH_SUBMITTED, LATEST_VALIDATION_SUBMITTED,
FALLBACK_VALIDATED, RRT_PATH_READY, then `source=RRT_CONNECT` on the regular
trajectory-send log. Planning profiles include planner_source and rrt_result.
FALLBACK_REJECTED explains failure/cancellation; it is never an execute command.

RViz tree recording is optional, bounded and independent of fallback admission.
No subscribers means no trace/FK work, but fallback search can still run. Blue/
red tree markers and the green baseline search path remain snapshot diagnostics,
not Motion authorization. Old trees clear on expiry/scene changes; the normal
Planned path display follows the manager's active path. A directly valid RRT
edge shows two roots without random tree growth. Workspace XYZ masking remains
visual-only and never becomes the robot collision checker.

## Verification

`test_rrt_fallback.py` exercises primary priority, recovery ordering, mailbox
ownership, fresh EMPTY, unknown-data rejection, target/start/scene changes,
deadlines, malformed output, no dependence on RViz, worker failures and the
normal RELEASE/EXECUTE checks. Actual MuJoCo collision tests reuse one model
fixture to avoid the known STL descriptor retention in the installed version.

`verify_rrt_fallback_simulation.py` runs actual PlanningNode, the installed C++
Motion component and the real RobotSimulation control/physics/feedback loop
on isolated ROS domain 228. Only the interactive viewer is replaced by a
headless context. The temporary roadmap has two nodes and zero edges, separated
by more than A*'s configured connector distance; A* is not mocked and collision
checking is not disabled. Target-box placement is prevalidated.

The probe requires RRT to reach the first joint goal and C++ Motion to report
COMPLETED. It then starts a return attempt and makes synthetic human messages
invalid during execution: STOP must occur with no subsequent EXECUTE/RELEASE.
It also subscribes to the RRT marker layers. No physical hardware or Kinect
pipeline is started; domain and simulation-mode checks are mandatory.

Add `--primary-delay-ms 1100` to reproduce callback-induced stale feedback after
the first real A* call. The probe delays returning that call; it does not fake
the search outcome or modify timeouts. It requires RRT_WAITING_FOR_FEEDBACK and
exactly one primary plan call before the first RRT goal completes. This mode
uses the same synthetic human input and real Motion/MuJoCo as the baseline.

```bash
ROS_DOMAIN_ID=228 ROS_LOCALHOST_ONLY=1 \
PYTHONPATH=src/planning:src/simulation:src/planning/test \
  python3 src/planning/test/verify_rrt_fallback_simulation.py \
  --output /tmp/cra_rrt_fallback_simulation.json
```

Recorded results and source hashes are in
`results/rrt_fallback_20261119.json`. Joint-goal error is an angle tolerance,
not a Cartesian target-distance or human-clearance measurement. Live Kinect,
moving-human detours during RRT execution, production full-graph failure
scenarios, interactive RViz and physical-robot execution remain unverified.
# A* scheduling selection

Primary graph search can run in `blocking` or `incremental` mode through
`planning.astar.mode` in YAML. Both keep graph-first RRT admission: a pending
slice is not a graph failure. See [ASTAR_EXECUTION_MODES.md](ASTAR_EXECUTION_MODES.md)
for configuration, snapshot ownership and verification limits.
