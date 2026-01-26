FROM python:3.10-slim

WORKDIR /app

# Install Python dependencies
COPY carla_scene_manager/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy proto file from gateway_bridge
COPY gateway_bridge/proto/vehicle_gateway.proto ./proto/

# Generate gRPC code
RUN python -m grpc_tools.protoc \
    -I./proto \
    --python_out=. \
    --grpc_python_out=. \
    --pyi_out=. \
    ./proto/vehicle_gateway.proto

# Copy application code
COPY carla_scene_manager/carla_scene_manager.py .

CMD ["python", "carla_scene_manager.py"]