import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ollie/git/1HoloAssist/ollie/ros2_ws/install/holoassist_servo_tools'
