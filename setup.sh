#!/bin/bash


# Activate the project's Python virtual environment
source /home/abh33/Documents/swarm_ros2_ws/.venv/bin/activate

# Source ROS 2
source /opt/ros/jazzy/setup.bash

export PYTHONPATH="$(python -c 'import site; print(site.getsitepackages()[0])'):$PYTHONPATH"

# Source the workspace (only if it has been built)
if [ -f /home/abh33/Documents/swarm_ros2_ws/install/setup.bash ]; then
    source /home/abh33/Documents/swarm_ros2_ws/install/setup.bash
fi

echo "✅ Swarm ROS 2 environment loaded."