# How to Use Simulation Launcher

Just as Gazebo Classic was able to launch the simulator from the ROS 2 launch file, this repository allows you to launch Isaac Sim using the simulation launcher.

## Use from Command Line

The simulation launcher can be run with the following command.

```bash
ros2 run isaac_ros2_scripts launcher
```

This command loads the default environment with the ground plane.

## Use from Launch File

The ROS 2 launch file can be used to load and launch any USD file.
An example of a launch file is shown below.

```python
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    core_stage_description_path = os.path.join(
        get_package_share_directory('stage_description'))

    core_stage_usd_path = os.path.join(stage_description_path,
                              'meshes', 'USD',
                              'stage.usd')

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher",
        parameters=[{'usd_path': str(core_stage_usd_path)}],
    )
    
    return LaunchDescription([
        isaac_launcher,
    ])
```

In the above example, the simulation launcher is launched by reading the file "stage_description/meshes/USD/stage.usd".