## Run the test node

cd ~/ros2_ws
colcon build --packages-select pd_test_node

source install/setup.bash


python3 src/pd_test_node/pd_test_node/pd_topic_test.py
or
src/pd_test_node/pd_test_node/pd_topic_test.