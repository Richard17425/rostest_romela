#!/bin/bash

# Automatically activate Python virtual environment and ROS 2 workspace

cd ~/Desktop/rostest-main/ros2_ws || exit

# Activate virtual environment if it exists
if [ -f envtest/bin/activate ]; then
    source envtest/bin/activate
else
    echo "❌ Virtual environment not found at envtest/bin/activate"
fi

# Source ROS 2 workspace setup if built
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "❌ ROS 2 setup.bash not found (did you build with colcon?)"
fi

echo "✅ Python virtual environment and ROS 2 workspace activated"
