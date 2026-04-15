import time

import numpy as np

from holoassist_movement.movement_sequence import MOVEMENT_SEQUENCE
from holoassist_movement.robot_demo_control import JOINT_NAMES


def run_demo(controller, duration):
    """Run the scripted movement sequence."""
    for index, step in enumerate(MOVEMENT_SEQUENCE, start=1):
        step_type = step.get('type')
        print(f"[APP] Step {index}: {step_type}")

        if step_type == 'move':
            joint_targets_deg = step['angles_deg']
            if len(joint_targets_deg) != len(JOINT_NAMES):
                raise ValueError(
                    f"Move step {index} must define {len(JOINT_NAMES)} joint angles"
                )
            target_joint_positions = np.deg2rad(joint_targets_deg).tolist()
            step_duration = float(step.get('duration', duration))
            controller.move_ur_joint_positions(
                target_joint_positions,
                duration=step_duration,
            )
        elif step_type == 'service':
            controller.call_service(step['name'])
        elif step_type == 'sleep':
            time.sleep(float(step['seconds']))
        else:
            raise ValueError(f"Unsupported step type in movement sequence: {step_type}")
