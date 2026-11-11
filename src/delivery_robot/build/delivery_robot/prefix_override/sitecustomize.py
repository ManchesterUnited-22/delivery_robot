import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lap/delivery_robot_ws/src/delivery_robot/install/delivery_robot'
