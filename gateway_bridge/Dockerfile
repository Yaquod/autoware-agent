FROM ghcr.io/autowarefoundation/autoware:latest-devel

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    pkg-config \
    libgrpc++-dev \
    libprotobuf-dev \
    protobuf-compiler-grpc \
    protobuf-compiler \
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS2 environment
RUN echo "source /opt/autoware/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

# Copy the application source code
WORKDIR /autoware/src
COPY . ./gateway_bridge

# Build the workspace
WORKDIR /autoware
RUN colcon build --symlink-install --packages-select gateway_bridge

# Set up the entrypoint to run our node
COPY ./gateway_bridge/ros-entrypoint.sh /
RUN chmod +x /ros-entrypoint.sh
ENTRYPOINT ["/ros-entrypoint.sh"]
CMD ["ros2", "run", "gnss_gateway_bridge", "gnss_gateway_bridge"]
