# How to Spawn Robot

This document describes how to place a robot into the running simulator.

Spawning is done by the `spawn_robot` node. It sends your URDF to the REST
API of the simulator launched beforehand (see
[How to Use Simulator Launcher](./how_to_use_simulator_launcher.md)), and the
simulator side then performs three steps in one request:

1. **Import the physical model** — the URDF is converted to USD with Isaac
   Sim's URDF importer and placed on the stage at the requested pose. The
   `isaac_drive_api`, `isaac_rigid_body` and `convex_decomposition` tags are
   applied here (see
   [Set up URDF for ros2_control](./setup_urdf_for_ros2_control.md)).
2. **Set up the robot controller** — the OmniGraph that exchanges
   `sensor_msgs/JointState` with ros2_control is generated from the
   `ros2_control` tag, and grippers/thrusters in the `isaac` tag are created.
3. **Set up the sensors** — LiDARs, cameras and contact sensors in the
   `isaac` tag are created together with their publisher graphs (see
   [Set up URDF for Sensors](./setup_urdf_for_sensors.md)).

## Spawning a robot from a launch file

A robot description is usually written as a xacro file, so expand it into a
URDF file first, then pass the file path to `spawn_robot`:

```python
    import xacro

    isaac_diffbot_description_path = os.path.join(
        get_package_share_directory('diffbot_description'))

    xacro_file = os.path.join(isaac_diffbot_description_path,
                              'robots',
                              'diffbot.urdf.xacro')
    urdf_path = os.path.join(isaac_diffbot_description_path, 'robots', 'diffbot.urdf')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    isaac_spawn_robot = Node(
        package="isaac_ros2_scripts",
        executable="spawn_robot",
        parameters=[{'urdf_path': str(urdf_path),
                    'x' : 0.0,
                    'y' : 0.0,
                    'z' : 0.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 1.57,
                    'fixed' : False,
                    }],
    )
```

Parameters of `spawn_robot`:

| Parameter | Meaning |
|----|----|
| `urdf_path` | Path to the URDF file (required) |
| `x`, `y`, `z` | Spawn position [m] |
| `R`, `P`, `Y` | Spawn orientation (roll / pitch / yaw) [rad] |
| `fixed` | `True`: fix the robot base to the world (for arm robots bolted to the environment) |
| `api_host`, `api_port` | Address of the simulator's REST API (default: `localhost:8080`) |

The node exits when the spawn is complete, so you can chain further actions
with `RegisterEventHandler` + `OnProcessExit` if needed. Since a single
request performs all three setup steps, no special ordering is required —
simply include `spawn_robot` in the same launch file as your ros2_control
nodes (see the
[mobile robot demo](../Demos/demo_for_mobile_robot.md) for a complete
example).

## Adding objects (USD assets) to the scene

Besides robots, you can add any USD asset (e.g. props, shelves, work pieces)
with the `add_usd` node:

```python
    isaac_add_usd = Node(
        package="isaac_ros2_scripts",
        executable="add_usd",
        parameters=[{'usd_path': str(usd_path),
                    'usd_name': 'target_object',
                    'x' : 1.0,
                    'y' : 0.0,
                    'z' : 0.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 0.0,
                    }],
    )
```

| Parameter | Meaning |
|----|----|
| `usd_path` | Path to the USD file (required) |
| `usd_name` | Name of the object on the stage; it is placed at `/World/<usd_name>` |
| `x` ... `Y` | Pose of the object |
| `api_host`, `api_port` | Address of the simulator's REST API |

## Publishing TF of an added object

To know where an added object is (e.g. to grasp it), the `publish_tf` node
sets up a TF publisher for a link of the object:

```python
    isaac_publish_tf = Node(
        package="isaac_ros2_scripts",
        executable="publish_tf",
        parameters=[{'robot_name': 'target_object',
                    'target_link': 'body_link',
                    }],
    )
```

| Parameter | Meaning |
|----|----|
| `robot_name` | The `usd_name` used in `add_usd` |
| `target_link` | Name of the link (prim) whose pose should be published as TF |
| `api_host`, `api_port` | Address of the simulator's REST API |

The pose is published on `/tf` on every physics step.
