#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace hvis det findes
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
else 
	echo "Ros setup not found!" >&2
	exit 1
fi

exec "$@"
