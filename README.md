# VehicleAutowareAgent

A ROS2-based autonomous vehicle agent that interfaces with the Autoware stack.
It manages trip lifecycle (localization, routing, engagement), exposes vehicle
state over gRPC, and is designed to run on embedded Linux targets.

---

## Overview

The agent sits between the Autoware autonomous driving stack and an external
gateway (mobile app, cloud backend, or fleet management system). It:

- Publishes initial pose and goal to Autoware
- Monitors localization, routing, and operation mode state
- Engages autonomous mode via the Autoware API
- Exposes a gRPC interface for trip commands and vehicle telemetry
- Parses a YAML route configuration to resolve GPS coordinates to lane IDs

The codebase is structured as a colcon workspace with ament_cmake build type,
using C++20, Boost.Asio for async dispatch, and protobuf/gRPC for the external
interface.

---

## Prerequisites

- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Autoware universe-devel dependencies

---

## 1. Install ROS2 Humble

Follow the official ROS2 installation guide exactly:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Quick summary:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install ros-humble-desktop python3-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 2. Install Autoware Dependencies

The Autoware message packages and overlays are required at build time.
Install the Autoware universe development image dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-autoware-vehicle-msgs \
  ros-humble-autoware-adapi-v1-msgs \
  ros-humble-autoware-planning-msgs \
  ros-humble-tier4-external-api-msgs \
  ros-humble-fastcdr
```

If those packages are not in the apt index on your machine, use the Autoware Docker image instead.

---

## 3. Install System Dependencies

```bash
sudo apt install -y \
  libgrpc++-dev libgrpc-dev \
  protobuf-compiler protobuf-compiler-grpc libprotobuf-dev \
  libyaml-cpp-dev libboost-system-dev \
  libgtest-dev libgmock-dev \
  ninja-build pkg-config python3-pip

pip3 install colcon-common-extensions vcstool
```

---

## 4. Build

```bash
# Clone the repo
git clone https://github.com/your-org/autoware-agent.git
cd autoware-agent

# Source ROS2 and Autoware
source /opt/ros/humble/setup.bash
~/

# Install remaining ROS2 deps from package.xml
rosdep install -y \
  --rosdistro=humble \
  --from-paths . \
  --ignore-src \
  --skip-keys="fastcdr grpc++ protobuf-dev libboost-system-dev geographiclib tier4_external_api_msgs"

# Build
colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --packages-select VehicleAutowareAgent

# Source the workspace
source install/setup.bash
```

---

## 5. Build Using Docker

This guarantees the same environment as CI and includes all Autoware packages.

```bash
# Pull the Autoware development image
docker pull ghcr.io/autowarefoundation/autoware:universe-devel

# Run a build container
docker run --rm \
  -v $(pwd):/workspace \
  -w /workspace \
  ghcr.io/autowarefoundation/autoware:universe-devel \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    source /opt/autoware/setup.sh && \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends \
      libgrpc++-dev libgrpc-dev \
      protobuf-compiler protobuf-compiler-grpc libprotobuf-dev \
      libyaml-cpp-dev libboost-system-dev libgtest-dev libgmock-dev && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      --packages-select VehicleAutowareAgent
  "
```

---
## Run Autoware

we are using Shinjuku-Map from AWSIM
https://tier4.github.io/AWSIM/Downloads/

```bash
# Set the map path
export MAP_PATH=~/nishishinjuku_autoware_map

# Launch Autoware (typical command structure)
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=$MAP_PATH \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
```

---

## 6. Run Tests

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test \
  --packages-select VehicleAutowareAgent \
  --event-handlers console_cohesion+

colcon test-result --verbose
```

---

## 7. Run the Agent

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run VehicleAutowareAgent autoware_agent \
  --map-path /path/to/route_config.yaml
```