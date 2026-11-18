# RRT observation without Motion dispatch

This is the next user-approved development stage after the offline clearance
experiment: run an observational candidate search in a separate process and
measure freshness, latest-scene rejection and callback responsiveness. GNG/A*
still supplies the live path. This does not activate RRT execution or best-first
retreat and does not change completed-task/idle avoidance policy.

## Data flow

1. After the existing A* and alternative-goal attempts fail specifically with
   `NO_GRAPH_PATH`, PlanningManager records a read-only attempt identifier,
   target generation, start and joint goal. Existing decisions remain intact.
2. A 50 ms node timer polls `ShadowRRTObserver`. It admits only stationary
   IDLE/COMPLETED/STOPPED contexts, fresh feedback/obstacles and known fresh
   OBSERVED human geometry. Active recovery, pending commands and faults block
   admission. The ordinary simulation UNKNOWN override does not blind this job.
3. The observer copies obstacle mocap poses, sizes, collision masks, immutable
   capsule tuples and their original received/expiry times. A spawned process
   owns a separate model, MuJoCo data and cache, then runs RRT and the approved
   clearance selector. There is only one in-flight job and no queued backlog.
4. A candidate with the same endpoints is submitted to the same worker for
   validation against a NEW copy of the latest scene. CRA's existing
   `prepare_execution_path` applies the actual segment and waypoint limits and
   collision certification at <=0.01 rad sampling. A new copy never extends the
   expiry of the old search snapshot.
5. The parent only labels a validation result current if the target/attempt
   remains applicable, feedback and geometry are fresh, the scene revision
   still matches and the validation start still equals measured joints. This
   is `VALID_FOR_CURRENT_SCENE_NO_DISPATCH`, not permission to execute later.
   The observer publishes only JSON diagnostics, never a trajectory or STOP.

The 0.01 rad admission movement limit only controls reuse of a start. It is not
a human-clearance allowance. A small change before latest validation replaces
the first path point and recertifies its connector. Any measured change after
validation prevents the current-scene label. Target changes are latched even
if the user returns the target to its earlier coordinates.

Each graph attempt is submitted once, with at least 3 s between searches.
While a stale job drains, the latest attempt can replace it at the next
admission; the process is not force-killed. Worker exceptions disable only the
observer. Shutdown cancels queued work and closes the executor without waiting
inside the ROS callback; a running cooperative job finishes before process exit.
An expensive native query can overrun a cooperative deadline, and late results
are rejected. This is not a hard real-time scheduling guarantee.

## Configuration and observation

`src/cra/config/robot.yaml`, under `planning.rrt_shadow`:

- `enabled: true`: observation in simulation mode only; false disables it.
- `search_budget: 0.5`, `selection_budget: 2.0`,
  `preference_search_budget: 0.8`, `validation_budget: 0.5`: wall seconds.
- `preferred_clearance: 0.07`: optional preference, original hard 0.05 m remains.
- `retry_interval: 3.0`: seconds between observational searches.

The human timeout remains 2 s, feedback gates and simulation flags remain
unchanged. A 2 s selection allowance does not guarantee sufficient time in a
2 s observation: capture age, cold process startup, initial RRT and final
validation also consume time. Expiry is a measured rejection, not suppressed
or refreshed to produce a successful experiment.

After building `planning` and `cra`, launch CRA normally and observe:

```bash
ros2 topic echo /planning/rrt_shadow
```

The same JSON appears as `RRT shadow:` in console output. Events include
`SEARCH_SUBMITTED`, `SKIPPED`, `LATEST_VALIDATION_SUBMITTED`, `DISCARDED`,
`RESULT` and `DISABLED`. Reasons distinguish stale feedback, unknown human,
changed target/start/attempt, search limits and latest validation failures.
Timing includes worker compute, capture-to-result delay, feedback age and
human age at scene capture/result. No event is an execution command. The module
does not create files or change the existing demo logging mode.

If no graph-search failure occurs, no shadow search is expected. A 3 cm start
under the 5 cm margin cannot be rescued by ordinary RRT. If existing recovery
has already become active, the shadow logs a skipped admission. None of these
cases proves that the whole free configuration space has no solution.

## Verification and limits

`test_rrt_shadow.py` covers isolation of copied sizes/masks/poses, bounded
admission, target/attempt replacement, start motion, real expiry, latest-scene
collision failure, broken workers, no-dispatch reports and a native spawned
process receiving feedback updates in the parent. Manager tests exercise the
actual observation hook without changing primary decisions.

`verify_rrt_shadow_ros.py` runs actual PlanningNode subscriptions, observer
timer, spawned worker and DDS publishers in isolated ROS domain 224, with a
synthetic far human and fixed joint feedback. It deliberately cancels the normal
A* timer and supplies an artificial failed graph attempt. It checks multiple
joint/human updates while the worker is active and zero Motion commands:

```bash
# In cra-humble, after sourcing ROS and the installed msgs package:
ROS_DOMAIN_ID=224 PYTHONPATH=src/planning:src/simulation:src/planning/test \
python3 src/planning/test/verify_rrt_shadow_ros.py \
  --output /tmp/cra_rrt_shadow_ros.json
```

Evidence is saved in `results/rrt_shadow_ros_20261118.json`. This is headless
ROS transport verification, not an end-to-end robot/camera demonstration.
All 204 Planning functional tests pass, including 22 new worker/coordinator/
manager tests. Planning/cra colcon build, new-file Flake8 and source-hash checks
pass. The saved DDS probe saw 36 joint and 36 human updates while the worker was
active, zero Motion commands and a successful latest-scene observational report.
The existing A* search, recovery searches and final live execution preparation
remain synchronous in their original timer. Only the new RRT observation work
is isolated. Do not infer that the complete Planning cycle is now nonblocking.
Next: collect real simulation failure/expiry/disposition timings before choosing
shared budgets, asynchronous primary planning or active RRT dispatch.

The first combined regression exceeded the container's 1024 open-file soft
limit during repeated model construction. An isolated probe measured 4 -> 74
descriptors after ten model loads, still 74 after releasing references and
garbage collection; descriptors pointed to STL assets. Coordinator tests now
reuse one native model because they test scheduling, while geometry/isolation
tests still construct independent models. The complete 204-test run then passes
without raising the limit. The worker also loads its model only once per process;
the external model-loading resource behavior is not repaired in this change.
