import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/biolab-linux/openARM/ros2_ws/install/widowx_control'
