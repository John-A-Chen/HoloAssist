#!/usr/bin/env python3
import time

import numpy as np
import roslibpy

JOINT_STATES_TOPIC = '/joint_states'
TRAJECTORY_TOPIC = '/scaled_joint_trajectory_controller/joint_trajectory'
JOINT_TRAJECTORY_TYPE = 'trajectory_msgs/msg/JointTrajectory'
JOINT_STATE_TYPE = 'sensor_msgs/msg/JointState'
UR_JOINT_ORDER = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

current_pos = None

def call_service(client, service_name):
    """Call a std_srvs/Trigger service and print the response."""
    service = roslibpy.Service(client, service_name, 'std_srvs/srv/Trigger')
    request = roslibpy.ServiceRequest({})  # Trigger service takes no arguments

    print(f"[ROS] Calling service: {service_name}")
    result = service.call(request)
    print(f"[ROS] Response: success={result['success']}, message='{result['message']}'")

def joint_state_cb(message):
    global current_pos
    if 'name' not in message or 'position' not in message:
        return

    if len(message['name']) != len(message['position']):
        return

    joint_map = {
        joint_name: joint_position
        for joint_name, joint_position in zip(message['name'], message['position'])
    }

    if not all(joint_name in joint_map for joint_name in UR_JOINT_ORDER):
        return

    current_pos = [joint_map[joint_name] for joint_name in UR_JOINT_ORDER]

def move_ur_joint_positions(client, joint_positions, duration=5.0):
    global current_pos
    current_pos = None

    try:
        # Subscribe to joint states to get the current position
        listener = roslibpy.Topic(client, JOINT_STATES_TOPIC, JOINT_STATE_TYPE)
        listener.subscribe(joint_state_cb)

        # Wait until we receive a joint state
        print("[ROS] Waiting for current joint state...")
        start_time = time.time()
        while current_pos is None and time.time() - start_time < 5.0:
            time.sleep(0.05)
        if current_pos is None:
            raise RuntimeError(f"No joint state received from {JOINT_STATES_TOPIC}")

        print(f"[ROS] Current joint positions: {current_pos}")

        # Build a JointTrajectory message for the ROS 2 scaled joint trajectory controller.
        trajectory_msg = {
            'joint_names': UR_JOINT_ORDER,
            'points': [
                {
                    'positions': current_pos,
                    'time_from_start': {'sec': 0, 'nanosec': 0}
                },
                {
                    'positions': joint_positions,
                    'time_from_start': {
                        'sec': int(duration),
                        'nanosec': int((duration - int(duration)) * 1e9)
                    }
                }
            ]
        }

        # Publish to the controller's /command topic
        topic = roslibpy.Topic(
            client,
            TRAJECTORY_TOPIC,
            JOINT_TRAJECTORY_TYPE
        )
        topic.advertise()
        topic.publish(roslibpy.Message(trajectory_msg))
        print(f"[ROS] Trajectory published to {TRAJECTORY_TOPIC}.")

        # Wait for motion to complete
        time.sleep(duration + 1.0)

        topic.unadvertise()
        listener.unsubscribe()

    finally:
        pass
        # print("[ROS] Disconnected from rosbridge.")

if __name__ == '__main__':
    client = roslibpy.Ros(host='127.0.0.1', port=9090)  # Replace with your ROS bridge IP
    client.run()
    try:
        # Example: open gripper, move, close gripper

        # move to a joint position
        target_joint_positions = np.deg2rad([36.10, -75.63, 68.76, -84.23, -88.24, 0.11]).tolist()
        move_ur_joint_positions(client, target_joint_positions, duration=7.0)

        # # Open gripper
        # call_service(client, '/onrobot/open')
        # time.sleep(2)

        # # Close gripper
        # call_service(client, '/onrobot/close')
        time.sleep(2)

        # move to a joint position
        target_joint_positions = np.deg2rad([-49.57, -92.18, -79.95, -93.72, 89.42, 0.0]).tolist()
        move_ur_joint_positions(client, target_joint_positions, duration=7.0)

        # # Open gripper
        # call_service(client, '/onrobot/open')
        # time.sleep(2)

        # # Close gripper
        # call_service(client, '/onrobot/close')
        time.sleep(2)

    except KeyboardInterrupt:
        print("\n[APP] Interrupted by user.")
