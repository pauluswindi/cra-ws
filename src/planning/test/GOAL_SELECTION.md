# Goal selection and bounded IK recovery

The primary roadmap is unchanged. Goal recovery supplies temporary joint-space
goals; it does not regenerate or append nodes/edges to the stored graph, and
never publishes Motion commands.

## Configuration

`src/cra/config/robot.yaml` now contains:

```yaml
planning:
  goal_selection:
    retain_valid_goal: true
    clearance_candidates: 4
    preferred_clearance: 0.07
    ik_fallback:
      enabled: true
      max_seeds: 6
      max_iterations: 80
      time_budget_ms: 100.0
      retry_interval: 1.0
      position_tolerance: 0.001
```

Restart the normal CRA launch after editing YAML. Goal recovery works with both
`planning.astar.mode: blocking` and `incremental`; the operator's selected mode
is preserved. Missing goal-selection configuration retains legacy nearest-goal
selection with IK disabled. New IK recovery is enabled only in simulation.

The Cartesian goal window remains the existing 4--12 cm distance from the
requested target. The 7 cm preferred human clearance is a soft ranking target,
not a new collision margin: the mandatory 5 cm human margin is unchanged.
The 1 mm IK position tolerance is convergence to an IK sample point, not a
human-clearance margin or Motion arrival tolerance.

## Selection and planning order

1. Recheck a previously successful goal against the current collision scene and
   Cartesian window. If valid, retain it to reduce unnecessary goal switching.
2. Otherwise, inspect the stored graph goals in Cartesian-distance order. Rank
   a bounded pool of valid goals by human clearance capped at the soft
   preference, then target distance. Collision checking supplies clearance
   during the existing state query; selection does not repeat human geometry
   queries just to rank or explain a rejection. Alternative-goal searches can
   continue past that first pool, up to their existing configured attempt limit.
3. Only when no valid retained/graph goal is available, recheck cached IK goals
   and optionally run bounded position IK. Different mathematical initial
   guesses and sample points within the unchanged goal window may produce
   another safe arm posture. Convergence alone is not acceptance: every result
   must satisfy joint limits, actual FK/window bounds and full state validity.
4. In `gng` and `combined`, try the selected goal through the existing A*
   connectors and roadmap first, including when its origin is IK. A valid
   graph route keeps source GNG_ASTAR. In `rrt`, use the shared selected goal
   directly for RRT path search.
5. With the default `combined` policy, only a completed NO_GRAPH_PATH batch may
   request simulation RRT fallback. `gng` disables RRT; `rrt` requests it directly
   after valid goal selection. No goal, IK nonconvergence, expired input or
   exhausted IK budget does not admit RRT. Human Retreat retains priority for
   an unsafe current pose. See [PLANNER_MODES.md](PLANNER_MODES.md).
6. Normal live-scene validation, STOP/RELEASE acknowledgements and final Motion
   checks still govern execution. A mathematical IK iterate is never a command.

The retained goal may have IK origin; retaining it does not run IK again. It
still uses graph search before any RRT fallback.

## IK implementation and computational cost

Planning uses position-only damped least-squares Jacobian IK with the same
step-norm limiting principle as `motion::RobotKinematics::inverseKinematics`.
World FK and the position Jacobian are transformed to CRA's robot base_link
frame using `FrameTransform`; Jacobian columns use MuJoCo joint DOF addresses.
The implementation leaves Motion's C++ solver and ROS interfaces unchanged.

The iteration phase has a soft 100 ms wall-time budget, up to six total seed/
point attempts and 80 iterations per attempt. A native call may overrun that
budget. Graph-goal scanning and bounded seed preparation are additional work;
the whole selection callback is not guaranteed to finish within 100 ms.
Incremental mode cooperatively yields during scanning and IK iteration.

Only four IK solutions for the current target are cached. They are revalidated
against fresh geometry before use; collision certificates are not cached across
scenes. Changing the target clears goal history and cached solutions. Failed IK
on an unchanged scene/start is not repeated. A changed scene must still respect
the one-second solver cooldown. A deferred cooldown can trigger one later
planning retry without requiring another camera pose change. No background IK
worker, new collision model or per-frame graph generation is introduced.

Incremental searches fork the small selector/cache state into the existing
private collision workspace. Results from cancelled target/start contexts are
not adopted. Snapshot deadlines and RRT input freshness requirements are
unchanged. New IK work requires fresh observed humans or a fresh certified
EMPTY when human avoidance is enabled; it cannot borrow the simulation's
startup UNKNOWN override.

## Diagnostics

The existing throttled `Planning profile` includes `goal_selection`. The regular
five-second `Planning snapshot` also prints `goal_reason` and `goal_origin`.
This is diagnostic evidence, not an additional high-rate topic.

Useful report fields are `window_candidates`, `checked`, `valid_graph`,
`valid_ik`, `valid_retained`, `rejected`, `graph_outcome`, `source`, `ik_attempts`,
`ik_cache_hits` and `ik_elapsed_ms`. `checked` includes retained/cached/IK
endpoint checks as well as graph checks. Rejection counts reuse the collision
filter's existing failure result; only three example rejections are retained.

Reasons distinguish NO_GRAPH_SAMPLES, GRAPH_GOALS_BLOCKED_HUMAN,
GRAPH_GOALS_BLOCKED_STATIC_OR_LIMITS, GRAPH_GOALS_REJECTED_MIXED, DATA_NOT_READY,
RETAINED_GOAL, GRAPH_GOAL_SELECTED, IK_INPUT_NOT_READY, IK_GOAL_SELECTED,
CACHED_IK_GOAL, IK_SCENE_ALREADY_TRIED, IK_RETRY_COOLDOWN, IK_BUDGET_EXHAUSTED
and IK_NO_VALID_GOAL. MUJOCO_CONTACT can include self/contact or static obstacle
contact; its example contains the actual geom pair. Solver exhaustion means no
accepted goal within configured attempts, not proof of physical impossibility.

## Verification limits

Tests cover frame/Jacobian agreement with native finite differences, known-FK
IK convergence and restoration, graph priority, clearance ranking, retained/
cached-goal revalidation, target invalidation, rejected graph goals, input loss,
budget/cooldown behavior, and graph-first IK/RRT policy in both scheduling modes.

The isolated native probe accepts `--goal-gap --astar-mode blocking` or
`incremental`. It removes the goal node from a temporary roadmap and tightens
only that fixture's graph connector limit, requiring an IK endpoint and then
RRT. Native Planning -> C++ Motion -> headless MuJoCo completes the first motion
and stops return motion when synthetic human input becomes invalid.
See `results/goal_recovery_simulation_20261124.json` and
`results/goal_recovery_blocking_simulation_20261124.json`.

These fixtures do not replay the user's 37-candidate moving-human scene, prove
live Kinect avoidance, or guarantee an IK solution. A genuinely blocked target
must still remain stopped if all candidate configurations are unsafe.
