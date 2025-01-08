import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rick-sanchez/Desktop/MASLAB-clown-penis/ros2_ws/install/fizzbuzz'
