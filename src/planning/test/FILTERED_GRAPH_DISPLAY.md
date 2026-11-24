# Retired whole-arm filtered graph display

The progressive `Joint graph - filtered` RViz display and its spawned worker
were removed on 2026-11-19 at the user's request because of their extra load.
Use [WORKSPACE_GRAPH_MASK.md](WORKSPACE_GRAPH_MASK.md) for the lightweight
replacement, which masks the original stored GNG in XYZ for display only.
Historical results under `results/filtered_graph_ros_20261118.json` are retained
as evidence of the retired implementation, not the current runtime.
