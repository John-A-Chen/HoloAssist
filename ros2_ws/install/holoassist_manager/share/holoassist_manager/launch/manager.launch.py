"""Launch the HoloAssist Manager node with default parameters."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('holoassist_manager'),
        'config',
        'manager_params.yaml',
    )

    manager_node = Node(
        package='holoassist_manager',
        executable='manager_node',
        name='holoassist_manager',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([manager_node])
