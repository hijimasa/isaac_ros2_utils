# ROS 2 utilities for Isaac Sim

This repository provides utilities to use [Isaac Sim](https://developer.nvidia.com/isaac-sim) like [Gazebo classic](https://classic.gazebosim.org/).

## Features

- This spawns URDF model at the desired timing and position.
- This provide ros2_control plugin to use ros2_controller, for example diff_drive_controller.
- This supports rotational and prismatic joints using position and velocity control.
- This sends joint status (position, velocity and effort) to ros2_control from Isaac Sim.
- This launches sensors from URDF description.
- This launchs sensors and controller at the desired timing.
- This sets stiffness, damping and friction from URDF description.

## Supported Versions

The `main` branch targets Isaac Sim 6.x with ROS 2 Jazzy.
For Isaac Sim 5.x and earlier, use the commits before the Isaac Sim 6 migration.

## System Requirements

| Element | Minimum Spec |
|----|--------------------|
| OS | Ubuntu 22.04 / 24.04 |
| CPU | Intel Core i7 (7th Generation) <br/> AMD Ryzen 5 |
| Cores | 4 |
| RAM | 32GB |
| Storage | 50GB SSD |
| GPU | GeForce RTX 2070 |
| VRAM | 8GB |
| NVIDIA Driver | 575 or later (Isaac Sim 6 is built with CUDA 12.9; with older drivers the RTX LiDAR publishes no data) |
| ROS 2 | Jazzy |

## How to Use

Please check [this document](https://hijimasa.github.io/isaac_ros2_utils/).

## Notes for Isaac Sim 6

- LiDAR: JSON lidar profiles were removed from Isaac Sim. Set the URDF
  `<isaac><sensor type="lidar"><config>` tag to one of the sensor asset names
  bundled with Isaac Sim (e.g. `Example_Rotary`, `Example_Rotary_2D`,
  `SICK_TIM781`, `RPLIDAR_S2E`, `OS1`, `HESAI_XT32_SD10`). Unsupported names
  fall back to `Example_Rotary_2D` (2D) / `Example_Rotary` (3D) with a warning
  that lists all supported names.
- Spawned robots place links under `/<robot>/Geometry/` and joints under
  `/<robot>/Physics/`, so sensor topic names derived from prim paths change
  accordingly.
- The surface gripper is built on the Isaac Sim 6 robot schema
  (an `IsaacSurfaceGripper` prim with a D6 attachment point joint) and is
  controlled through the `SurfaceGripperInterface`.

