import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sebastian/git/rs2/HoloAssist/ros2_ws/install/holoassist_manager'
