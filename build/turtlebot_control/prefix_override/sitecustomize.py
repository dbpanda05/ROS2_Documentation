import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/debarghya/ros2_ws/src/install/turtlebot_control'
