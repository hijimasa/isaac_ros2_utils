# Demo For Mobile Robot

![demo movie](../figs/shm_movie-2023-08-05_13.14.29.gif)

1. Install Docker and pull [Isaac Sim Docker Image](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_container.html).

2. Clone the repo to your ros2 workspace<br/>
   ```bash
   git clone https://github.com/hijimasa/isaac-ros2-control-sample.git
   ```

3. Get git submodules<br/>
   ```bash
   cd isaac-ros2-control-sample
   git submodule update --init --recursive
   ```

4. Build a docker image with shell script.<br/>
   ```bash
   cd docker
   ./build_docker_image.sh
   ```

5. Launch a docker container<br/>
   ```bash
   ./launch_docker.sh
   ```

6. Build ros2 source codes<br/>
   ```bash
   colcon build && source install/setup.bash
   ```

7. Launch simulator<br/>
   ```bash
   ros2 run isaac_ros2_scripts launcher
   ```

8. Spawn robot (another terminal)<br/>
   ```bash
   docker exec -it isaac-sim /bin/bash
   ros2 launch isaac_diffbot_sim diffbot_spawn.launch.py
   ```

9. Launch teleop_twist_keyboard (another terminal)<br/>
   ```bash
   docker exec -it isaac-sim /bin/bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   
> [!NOTE]
> For the first time, launching Isaac Sim takes a very long time.
> Isaac Sim must be fully launched to spawn the robot.
