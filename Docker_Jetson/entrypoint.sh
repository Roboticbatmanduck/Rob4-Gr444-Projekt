#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/Humble/setup.bash

# Source workspace hvis det findes
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

exec "$@"