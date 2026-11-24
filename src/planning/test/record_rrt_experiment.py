"""Record equal-endpoint offline RRT examples; no ROS or robot execution.

Freeze human freshness for this geometric experiment only. Search deadlines
still use real wall time. JSON results are historical, never live certificates.
"""

import argparse
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

from benchmark_rrt_connect import GOAL, MODEL, ROOT, START, make_scenes, planner
from planning.algorithms.rrt_connect import RRTConnectPlanner
from planning.rrt_trace import SearchTrace
from planning.visualization.projection import project_configuration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=9)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with patch('time.monotonic', return_value=100.):
        scene = make_scenes()[1]
        for name, enabled in (('without_human', False), ('with_human', True)):
            p = planner(scene[1])
            c = p.collision_filter
            c.enable_human_obstacles(enabled)
            trace = SearchTrace()
            search = RRTConnectPlanner(c, trace=trace, time_budget=5.)
            path = search.plan(START, GOAL, seed=args.seed)
            if path is not None:
                assert c.is_path_valid(path)
            data = trace.project(lambda q: project_configuration(
                c.model, c.data, c.qpos_addresses, c.base_id,
                c.model.body('end_effector').id, q))
            data.update(frame_id='base_link', reason=search.last_plan_failure,
                        scene=name, historical=True, frozen_freshness=True, seed=args.seed,
                        start=START.tolist(), goal=GOAL.tolist(), joint_names=p.joint_names,
                        scene_revision=c.environment_revision,
                        human_capsules=c.human_capsules if enabled else [],
                        statistics=search.last_statistics,
                        source_hashes={str(path.relative_to(ROOT)): hashlib.sha256(
                            path.read_bytes()).hexdigest() for path in (
                                MODEL, ROOT / 'src/planning/planning/algorithms/rrt_connect.py',
                                ROOT / 'src/planning/planning/algorithms/collision_filter.py',
                                ROOT / 'src/planning/planning/rrt_trace.py')})
            destination = args.output_dir / (name + '.json')
            destination.write_text(json.dumps(data, allow_nan=False, indent=2) + '\n')
            print(f'{name}: {search.last_plan_failure}, '
                  f"nodes={search.last_statistics['nodes']}, "
                  f'rejections shown={len(trace.rejected)}, saved {destination}', flush=True)


if __name__ == '__main__':
    main()
