#!/bin/bash
export __NV_PRIME_RENDER_OFFLOAD=1 && export __GLX_VENDOR_LIBRARY_NAME=nvidia

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export TURTLEBOT3_MODEL=waffle

#export CYCLONEDDS_URI=file:///workspace/install/robot_description/share/robot_description/config/cyclonedds_config.xml
# Initialize environment
source /opt/ros/humble/setup.bash

# Change to workspace
cd /workspace

# Execute command (preserves Ctrl+C handling)
exec "$@"
