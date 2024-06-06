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

## System Requirements

| Element | Minimum Spec |
|----|--------------------|
| OS | Ubuntu 22.04 |
| CPU | Intel Core i7 (7th Generation) <br/> AMD Ryzen 5 |
| Cores | 4 |
| RAM | 32GB |
| Storage | 50GB SSD |
| GPU | GeForce RTX 2070 |
| VRAM | 8GB |
| ROS 2 | Humble |

## How to Use

Please check [this document](https://hijimasa.github.io/isaac_ros2_utils/).

