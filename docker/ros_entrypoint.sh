#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Set up tab completion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

# Print welcome message
echo "==================================="
echo "Vehicle ROS2 Development Environment"
echo "==================================="

# Execute whatever command was passed in
exec "$@"