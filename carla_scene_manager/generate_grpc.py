"""
Generates the gRPC Python stubs from the vehicle_gateway.proto file.
"""
from grpc_tools import protoc
import os

def generate():
    """Generates the gRPC code."""
    proto_path = '../proto'
    output_path = '.'
    proto_file = os.path.join(proto_path, 'vehicle_gateway.proto')

    if not os.path.exists(proto_file):
        print(f"Error: Proto file not found at {proto_file}")
        return

    print(f"Generating gRPC code from {proto_file} to {output_path}")

    # Use protoc with explicit protobuf version compatibility
    result = protoc.main((
        '',
        f'-I{proto_path}',
        f'--python_out={output_path}',
        f'--grpc_python_out={output_path}',
        f'--pyi_out={output_path}',  # Generate type stubs
        proto_file,
    ))

    if result == 0:
        print("gRPC code generated successfully.")
    else:
        print(f"Error generating gRPC code. Exit code: {result}")

if __name__ == '__main__':
    generate()