import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tan/ros2_ws/src/sutd_ws/install/sutd_ws'
