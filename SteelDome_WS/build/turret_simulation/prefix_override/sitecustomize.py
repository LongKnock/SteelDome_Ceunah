import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/orto03/ROS_Workspace/SteelDome_Project/SteelDome_WS/install/turret_simulation'
