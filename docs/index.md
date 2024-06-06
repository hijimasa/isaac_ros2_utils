# Introduction

![demo movie](./figs/shm_movie-2023-06-13_21.52.52.gif)

[This repository](https://github.com/hijimasa/isaac_ros2_utils) provides utilities to use [Isaac Sim](https://developer.nvidia.com/isaac-sim) like [Gazebo classic](https://classic.gazebosim.org/).

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
| GPU | GeForce RTX 3070 |
| VRAM | 8GB |
| ROS 2 | Humble |

Please refer to [Isaac Sim System Requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements).

## Demo

[This sample repository](https://github.com/hijimasa/isaac-ros2-control-sample) provides demonstlations for mobile robot and arm robot.

Please check following documents.

[For Mobile Robot](./Demos/demo_for_mobile_robot.md)

[For Arm Robot](./Demos/demo_for_arm_robot.md)

## How to Use

Check [this tutorial](./Tutorials/tutorial.md).
        
## LICENSE

This repository is provided with MIT license.
Please refer to [this](./LICENSE.md).