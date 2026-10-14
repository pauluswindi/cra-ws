# Workspace filter patch

## Files

Copy the three Python files under `scripts/planning/` into the existing `workspace_planning` package:

- `workspace_filter.py` (new)
- `a_star.py` (updated to support valid-node and valid-edge masks)
- `planning_node.py` (updated to use the filter and subscribe to the obstacle marker)

The existing `scripts/workspace/frame_transform.py` is used without modification.

## Behavior

- Target geometry is loaded from the MuJoCo XML specified by `robot.model.path` in `config/robot.yaml`, using the body name `target` and base frame returned by `RobotConfig`.
- The target mesh is represented by its conservative axis-aligned bounding box in `base_link`. The box follows `/planning/target_position`; the geometry size continues to come from XML.
- One dummy obstacle is received on `/workspace/obstacle` as `visualization_msgs/msg/Marker`.
- Supported obstacle marker types: `CUBE`, `SPHERE`, and `CYLINDER`.
- Obstacle pose and scale must use the `base_link` frame and meters.
- Invalid nodes and edges are filtered dynamically. Original GNG positions and edge indices are retained.
- Valid GNG nodes are shown on `/planning/gng_nodes`, blocked nodes on `/planning/blocked_gng_nodes`, valid edges on `/planning/gng_edges`, and the target bounding box on `/planning/target_bounds`.

## Publish or move the dummy obstacle

Example: publish a box at (0.25, 0.10, 0.35), size (0.10, 0.15, 0.20) meters:

```bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker \
"{header: {frame_id: base_link}, ns: workspace_obstacle, id: 0, type: 1, action: 0, pose: {position: {x: 0.25, y: 0.10, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, scale: {x: 0.10, y: 0.15, z: 0.20}, color: {r: 1.0, g: 0.1, b: 0.1, a: 0.7}}"
```

Publish another `Marker` with the same `ns` and `id` but new position/scale values to move or resize it. Remove it from the filter with:

```bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker \
"{header: {frame_id: base_link}, ns: workspace_obstacle, id: 0, action: 2}"
```

## Test core filtering without ROS

From the patch directory:

```bash
python3 -m unittest discover -s tests -v
```

The ROS integration must additionally be tested in the project's ROS 2 environment, where MuJoCo and the model XML/assets are installed.

## Limitations in this first version

- The mesh target is filtered using an axis-aligned bounding box. This is conservative and may remove some free workspace around a non-box-shaped target.
- Target body orientation changes are not tracked because `/planning/target_position` carries only a `Point`. Target position changes are tracked; the target geometry and its orientation assumptions come from the model at startup.
- The endpoint segment to the target is allowed to enter the target's own bounding box intentionally, but it is still checked against the dummy obstacle.
- This filters the EoF workspace graph, not every robot link. Full link-level collision checking remains a later development stage.
