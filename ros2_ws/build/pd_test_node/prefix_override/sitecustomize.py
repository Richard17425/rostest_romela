import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ruiqimao/Desktop/rostest-main/ros2_ws/install/pd_test_node'
