"""Controlled scheduling comparison, not a live-camera or FPS benchmark.

Use the real roadmap/model, frozen far-human geometry and a virtual 10 ms
timer. Collision work uses the native checker and real perf_counter budgets.
No ROS callbacks, Motion commands or camera devices are involved.
"""

import argparse
import contextlib
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

from benchmark_rrt_connect import CONFIG, FAR, GRAPH, MODEL, ROOT, configure
from planning.astar_execution import GraphSearch, GraphSearchWorkspace, astar_settings
from planning.path_planner import PathPlanner
from simulation.frame_transform import FrameTransform


def run(case):
    now = [100.]
    with patch('time.monotonic', side_effect=lambda: now[0]):
        planner = PathPlanner(GRAPH, MODEL, CONFIG)
        checker = planner.collision_filter
        configure(checker, FAR)
        target_world = checker.data.mocap_pos[checker.obstacle_geoms['target']['mocap_id']]
        target = FrameTransform.mujoco_base_to_robot_base(
            checker.data.xmat[checker.base_id].reshape(3, 3).T
            @ (target_world - checker.data.xpos[checker.base_id]))
        start = np.zeros(5)
        if case == 'replan':
            # Recorded robot/target values, but reconstructed FAR-human geometry:
            # this exercises a harder route and is not a replay of the live scene.
            start = np.array([1.3624008556443548, -.8270398164196338, 1.3273130924385779,
                              -.19822505544778982, .03495262485304699])
            target = np.array([.1532060447883779, -.09999816339340062, .3505738206671457])
            target_world = (checker.data.xpos[checker.base_id]
                            + checker.data.xmat[checker.base_id].reshape(3, 3)
                            @ FrameTransform.robot_base_to_mujoco_base(target))
            planner.update_obstacle('target', target_world)
        workspace = GraphSearchWorkspace(planner)
        trials, routes = [], []
        for cap in (256, 2048):
            now[0] = 100.
            checker.set_human_capsules(FAR)
            settings = astar_settings({'astar': {
                'mode': 'incremental', 'max_operations_per_slice': cap}})
            job = GraphSearch(workspace, checker, start, target, 1, .12, .04, settings)
            next_tick = now[0] + settings['slice_period_ms'] / 1000.
            while not job.done and now[0] < 103.:
                now[0] = next_tick
                job.advance()
                # An indivisible query may overrun its period. A virtual timer
                # must not claim less elapsed time than the measured work itself.
                now[0] += job.last_slice_ms / 1000.
                next_tick = max(now[0], next_tick + settings['slice_period_ms'] / 1000.)
            if job.error is not None:
                raise job.error
            stats = job.statistics()
            # Timer ticks are virtual; work_ms is measured actual callback work.
            stats['virtual_elapsed_ms'] = stats.pop('elapsed_ms')
            assert stats['virtual_elapsed_ms'] + .01 >= stats['work_ms']
            trials.append(dict(settings=settings, planned=bool(job.planned), **stats))
            routes.append(None if not job.planned else job.planner.current_path.copy())
            job.close()
        equal = (bool(np.array_equal(*routes)) if all(p is not None for p in routes) else None)
        if equal is False:
            raise AssertionError('Operation cap changed the route in an identical scene')
    files = [Path(__file__), CONFIG, GRAPH, MODEL,
             ROOT / 'src/planning/planning/astar_execution.py',
             ROOT / 'src/planning/planning/algorithms/a_star.py',
             ROOT / 'src/planning/planning/path_planner.py']
    return dict(scope=__doc__, case=case, human_snapshot=asdict(FAR), start=start.tolist(),
                target_base=target.tolist(), roadmap_nodes=len(planner.joint_q),
                roadmap_edges=len(planner.joint_edges), trials=trials,
                equal_routes_when_both_found=equal,
                source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(
                    p.read_bytes()).hexdigest() for p in files})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--case', choices=('default', 'replan'), default='replan')
    args = parser.parse_args()
    with contextlib.redirect_stdout(io.StringIO()):
        report = run(args.case)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('trials', 'equal_routes_when_both_found')},
                     indent=2))


if __name__ == '__main__':
    main()
