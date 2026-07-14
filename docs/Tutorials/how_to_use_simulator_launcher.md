# How to Use Simulation Launcher

Just as Gazebo Classic can be launched from a ROS 2 launch file, this
repository lets you launch Isaac Sim with the simulation launcher node.

The launcher starts Isaac Sim (which takes several minutes on the first run),
loads a stage (a USD file describing the environment), and starts a small
**REST API server** inside Isaac Sim. Robots and objects are later added
through this API — see [How to Spawn Robot](./how_to_spawn_robot.md).

## Use from Command Line

```bash
ros2 run isaac_ros2_scripts launcher
```

This command loads the default environment with the ground plane.

Once "Simulation ready" appears in the log, the REST API is available at
`http://localhost:8080`. Press the **Play** button in the Isaac Sim window
(or `POST /simulation/play`) to start the physics simulation.

## Parameters

| Parameter | Default | Meaning |
|----|----|----|
| `usd_path` | ground plane stage | USD file to load as the environment |
| `fps` | 60.0 | Rendering / physics update rate of the simulator [Hz] |
| `time_steps_per_second` | 600.0 | Physics solver steps per second. Larger values make contact and joint simulation more stable but heavier |
| `real_fps` | same as `fps` | How often the simulation is updated in wall-clock time [Hz]. Setting it lower than `fps` slows the simulation down relative to real time |
| `api_port` | 8080 | Port of the REST API server |
| `isaac_path` | `/isaac-sim` | Isaac Sim installation directory |

## Launcher variants

| Executable | Behavior |
|----|----|
| `launcher` | Standard: gravity 9.81 m/s², GUI |
| `launcher_with_headless` | Same as `launcher` but without the GUI window |
| `launcher_zero_g` | Zero gravity (for space / underwater robots) |
| `launcher_with_reset` | Adds a shared-memory trigger that stops and restarts the simulation timeline (used to reset the world without relaunching Isaac Sim) |

## Use from Launch File

The ROS 2 launch file can load any USD file as the environment:

```python
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    stage_description_path = os.path.join(
        get_package_share_directory('stage_description'))

    stage_usd_path = os.path.join(stage_description_path,
                              'meshes', 'USD',
                              'stage.usd')

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher",
        parameters=[{'usd_path': str(stage_usd_path)}],
    )

    return LaunchDescription([
        isaac_launcher,
    ])
```

In the example above, the launcher starts Isaac Sim with the environment
"stage_description/meshes/USD/stage.usd".

## REST API

The launcher exposes the following endpoints (Swagger UI is available at
`http://localhost:8080/docs`). Usually you do not call them directly —
the `spawn_robot` / `add_usd` / `publish_tf` nodes described in
[How to Spawn Robot](./how_to_spawn_robot.md) call them for you.

| Endpoint | Method | Meaning |
|----|----|----|
| `/health` | GET | Health check |
| `/spawn_robot` | POST | Import a URDF and set up its controller and sensors |
| `/add_usd` | POST | Add a USD asset to the stage |
| `/publish_tf` | POST | Publish TF for a link of an added asset |
| `/simulation/play` | POST | Start the simulation |
| `/simulation/pause` | POST | Pause the simulation |
| `/simulation/stop` | POST | Stop the simulation |

> **Note:** for the first time, launching Isaac Sim takes a very long time
> (shaders are compiled and assets are downloaded). Isaac Sim must be fully
> launched before a robot can be spawned.
