from glob import glob

from setuptools import find_packages, setup

package_name = 'holoassist_movement'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='john',
    maintainer_email='cohnjhen@gmail.com',
    description='Robot movement demos and motion helper scripts for HoloAssist.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_joints = holoassist_movement.move_to_joints:main',
            'robot_state = holoassist_movement.robot_state:main',
            'robot_demo_control = holoassist_movement.robot_demo_control:main',
        ],
    },
)
