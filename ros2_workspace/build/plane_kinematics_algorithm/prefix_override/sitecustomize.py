import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rodding/ros2_workspace/install/plane_kinematics_algorithm'
