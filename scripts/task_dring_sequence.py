"""Single-side task identity; historical ten-stage plans are read-only."""
SCHEMA='dring_single_sweep_two_finger_v2'
CENTER_SCHEMA='index_center_check_v1'
CENTER_LABELS=('Hand index_only','Center approach','Center check')


def is_center_check(plan):
    return bool(plan and plan.get('task_schema')==CENTER_SCHEMA)


def labels(direction):
    if direction not in ('left','right'):raise ValueError('Unknown D-ring sweep direction')
    return ('Approach','Hand dring_two_finger','Insert',direction.title()+' 120',
            'Withdraw','Hand open','Return ready')


def plan_labels(plan):
    if plan.get('task_schema')==CENTER_SCHEMA:return CENTER_LABELS
    if plan.get('task_schema')!=SCHEMA:
        raise ValueError('Old D-ring task retired; replan a single left or right sweep')
    return labels(plan.get('sweep_direction'))
