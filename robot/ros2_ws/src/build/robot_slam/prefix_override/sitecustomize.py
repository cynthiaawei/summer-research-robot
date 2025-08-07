import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/krrish/summer-research-robot/robot/ros2_ws/src/install/robot_slam'
