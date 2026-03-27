"""Editable movement sequence definitions for the robot demo."""

DEFAULT_MOVE_DURATION = 7.0
DEFAULT_GRIPPER_DELAY = 2.0

# Edit this list to define the order of robot actions.
# Supported step types:
# - {'type': 'move', 'angles_deg': [j1, j2, j3, j4, j5, j6]}
# - {'type': 'service', 'name': '/onrobot/open'}
# - {'type': 'sleep', 'seconds': 2.0}
MOVEMENT_SEQUENCE = [
    {
        'type': 'move',
        'angles_deg': [36.10, -75.63, 68.76, -84.23, -88.24, 0.11],
    },
    {
        'type': 'service',
        'name': '/onrobot/open',
    },
    {
        'type': 'sleep',
        'seconds': DEFAULT_GRIPPER_DELAY,
    },
    {
        'type': 'service',
        'name': '/onrobot/close',
    },
    {
        'type': 'sleep',
        'seconds': DEFAULT_GRIPPER_DELAY,
    },
    {
        'type': 'move',
        'angles_deg': [-49.57, -92.18, -79.95, -93.72, 89.42, 0.0],
    },
    {
        'type': 'service',
        'name': '/onrobot/open',
    },
    {
        'type': 'sleep',
        'seconds': DEFAULT_GRIPPER_DELAY,
    },
    {
        'type': 'service',
        'name': '/onrobot/close',
    },
    {
        'type': 'sleep',
        'seconds': DEFAULT_GRIPPER_DELAY,
    },
]
