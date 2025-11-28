# CARLA Server Setup

## One-time Setup

Build the image:
```bash
docker build -f Dockerfile.carla -t carla:0.9.15 .
```

Create network:
```bash
docker network create autoware-carla-net
```

## Start CARLA

```bash
docker stop carla-server 2>/dev/null || true && \
docker rm carla-server 2>/dev/null || true && \
docker run -d \
  --name carla-server \
  --network autoware-carla-net \
  --runtime=nvidia \
  --gpus all \
  -p 2000-2002:2000-2002 \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  carla:0.9.15 \
  bash CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000 -quality-level=Low
```

## Common Operations

View logs:
```bash
docker logs -f carla-server
```

Stop server:
```bash
docker stop carla-server
```

Restart server:
```bash
docker restart carla-server
```

Check if running:
```bash
docker ps | grep carla-server
```

Remove container:
```bash
docker rm -f carla-server
```

## Connection

- RPC Port: 2000
- Streaming Port: 2001-2002
- Network: autoware-carla-net

## Typical Workflow

1. Start CARLA (command above)
2. Wait 10-15 seconds for initialization
3. Connect your application to localhost:2000
