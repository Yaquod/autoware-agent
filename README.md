# CARLA Gateway Bridge

This repository contains a ROS2-based gateway bridge for CARLA, along with a scene manager and other utilities.

## Repository Structure

* `gateway_bridge/`: A ROS2 package that acts as a bridge between CARLA and ROS2, allowing communication and data
  transfer.
* `carla_scene_manager/`: A Python package for managing scenes and actors in CARLA.
* `autoware_map/`: Contains map data for Autoware.
* `docker-compose.yml`: Docker Compose file for running the CARLA server and other services.
* `autoware_bridge.Dockerfile`: Dockerfile for building the Autoware bridge.

## Usage

To run the complete setup, use the following command:

```bash
docker-compose up --build
```
