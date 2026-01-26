FROM ghcr.io/autowarefoundation/autoware:latest-devel-arm64

# Install gRPC and Protobuf development libraries needed for building the bridge
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    libgrpc++-dev \
    libprotobuf-dev \
    protobuf-compiler-grpc && \
    sudo rm -rf /var/lib/apt/lists/*