#!/usr/bin/env bash
#
# Runtime entrypoint for the vehicle_autoware_agent image.
# Sources ROS 2, the Autoware overlay, and the agent workspace, then execs the
# agent node (or whatever command is passed to `docker run` / compose `command`).

set -euo pipefail

source /opt/ros/humble/setup.bash

# Autoware overlay (present because the runtime image is FROM the Autoware image).
if [ -f /opt/autoware/setup.bash ]; then
  source /opt/autoware/setup.bash
fi

# Agent colcon install space.
if [ -f /opt/agent/install/setup.bash ]; then
  source /opt/agent/install/setup.bash
fi

# Make the source-built protobuf/gRPC/zenoh shared libs discoverable.
export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"

# Default command: run the agent. Overridable via compose `command:` / `docker run`.
if [ "$#" -eq 0 ]; then
  set -- ros2 run vehicle_autoware_agent autoware_agent
fi

exec "$@"
