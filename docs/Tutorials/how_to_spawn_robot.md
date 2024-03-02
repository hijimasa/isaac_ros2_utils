# How to Spawn Robot

This document describes the procedure for generating a robot on the simulator.
The generation of a robot model can be divided into the following three major steps: 1.

1. generation of the physical model of the robot
2. generation of the robot's sensor model
3. generation of the robot controller

The three steps are described in detail below.

## Generate the physical model of the robot

The following is an example of creating a node to generate a physical model of a robot.

```python
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

As shown in the example above, spawn_robot from the isaac_ros2_scripts package is used to generate the physical model.
spawn_robot requires as arguments the URDF file of the robot to be generated, its position (X,Y,Z) and orientation (Roll,Pitch,Yaw), and whether the robot is fixed or not.
The argument if the robot is fixed or not is used if the arm robot is fixed to the environment.

## Generate sensor model of the robot

The following is an example of creating a node to generate a sensor model of a robot.

```python
    isaac_prepare_sensors = Node(
        package="isaac_ros2_scripts",
        executable="prepare_sensors",
        parameters=[{'urdf_path': str(urdf_path)}],
    )
```

Since the generation of the robot's sensor model requires the information in the isaac tag in the URDF file, prepare_sensors requires the URDF file of the robot to be generated as an argument.


## Generate robot controllers

The following is an example of creating a node to generate a controller for a robot.
The controller here refers to the internal controller that drives the robot on the simulator based on shared memory values, not ros2_controller.

```python
    isaac_prepare_robot_controller = Node(
        package="isaac_ros2_scripts",
        executable="prepare_robot_controller",
        parameters=[{'urdf_path': str(urdf_path)}],
    )
```

The information in the ros2_control tag in the URDF file is needed to generate the robot's controller, so prepare_robot_controller requires the URDF file of the robot to be generated as an argument.

## How to start up a node

To generate additional robot models for Isaac Sim once it is up and running, this repository uses the Python REPL extension for Isaac Sim.
This extension cannot be given multiple processes at the same time, so when launching a node, use RegisterEventHandler to process the nodes in order.

```python
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_spawn_robot,
                on_exit=[isaac_prepare_sensors],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_prepare_sensors,
                on_exit=[isaac_prepare_robot_controller],
            )
        ),
        isaac_spawn_robot,
    ])
```
