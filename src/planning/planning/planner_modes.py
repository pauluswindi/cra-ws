"""Select the primary path-search policy independently of worker settings."""


def configured_planner(planning):
    mode = planning.get('planner_mode')
    if 'planner_mode' in planning and mode not in ('gng', 'rrt', 'combined'):
        raise ValueError('planning.planner_mode must be gng, rrt or combined')
    return mode


def planner_selection(planning, rrt_override=''):
    """Keep old shadow/off probe overrides and legacy YAML working.

    The public YAML switch owns normal execution. Explicit node overrides are
    retained for older diagnostic probes and their effective mode is logged.
    """
    mode = configured_planner(planning)
    if rrt_override:
        if rrt_override not in ('off', 'shadow', 'fallback'):
            raise ValueError('Invalid diagnostic rrt_mode override')
        if rrt_override == 'fallback':
            return ('rrt' if mode == 'rrt' else 'combined'), rrt_override
        return 'gng', rrt_override
    if mode is None:
        return None, ''
    return mode, 'off' if mode == 'gng' else 'fallback'
