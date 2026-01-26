#!/bin/bash
set -e

# Source ROS2 and Autoware environment
source /opt/autoware/setup.bash

# Source the local workspace
source /autoware/install/setup.bash

# Execute the command passed to this script
exec "$@"