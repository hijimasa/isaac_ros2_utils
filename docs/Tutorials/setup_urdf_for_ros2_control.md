# Set up URDF for ros2_control

This document explains how to prepare your robot's URDF so that it can be
driven through [ros2_control](https://control.ros.org/) on Isaac Sim, and
what isaac_ros2_utils does with each part of the description behind the
scenes.

## Introduction to ros2_control

ros2_control offers developers a common API that allows your software to
switch between many different robot types (and simulators) by simply changing
a plugin in the URDF. The controller side (e.g. `diff_drive_controller`,
`joint_trajectory_controller`) stays exactly the same; only the *hardware
interface* that talks to the actual robot or simulator is replaced.

To use ros2_control with Isaac Sim, this repository uses
[topic_based_ros2_control](https://github.com/hijimasa/topic_based_ros2_control).
This hardware interface exchanges `sensor_msgs/JointState` messages with the
simulator over two ROS 2 topics:

- It **subscribes** to a joint states topic. Isaac Sim publishes the actual
  position / velocity / effort of every joint there.
- It **publishes** to a joint commands topic. Isaac Sim subscribes to it and
  forwards the commands to the joint drives.

The topic names are declared as hardware parameters:

```xml
<ros2_control name="diffbot" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/diffbot/joint_command</param>
    <param name="joint_states_topic">/diffbot/joint_states</param>
    <param name="sum_wrapped_joint_states">true</param>
  </hardware>
  ...
</ros2_control>
```

**How isaac_ros2_utils handles this:** when the robot is spawned,
`robot_controller.py` reads the `joint_commands_topic` and
`joint_states_topic` parameters from the URDF and builds an OmniGraph
(a node network evaluated on every physics step) inside Isaac Sim:

```
OnPhysicsStep ─┬─> ArticulationState ──> ROS2Publisher  ──> joint_states_topic
               └─> ROS2Subscriber(joint_commands_topic) ──> ArticulationController
```

So from the ROS 2 side, Isaac Sim looks like a piece of hardware that reports
its joint states and accepts joint commands — which is exactly what
`TopicBasedSystem` expects.

Available hardware parameters:

| Parameter | Meaning |
|----|----|
| `joint_commands_topic` | Topic that Isaac Sim receives joint commands on |
| `joint_states_topic` | Topic that Isaac Sim publishes joint states on |
| `sum_wrapped_joint_states` | Set `true` for robots with continuous joints (e.g. wheels). Isaac Sim reports wrapped angles in the range of ±2π; with this option the hardware interface accumulates them into a continuous angle, which controllers like `diff_drive_controller` require for odometry. |
| `trigger_joint_command_threshold` | Publish a command message only when the command changes by more than this value (optional). |

> **Note:** the joint state topic is tied to the topic names in the URDF, not
> to the robot name. When operating multiple robots, give each robot its own
> topic names (e.g. prefix them with the robot name).

## Set up Joint Information

Inside the `ros2_control` tag, list every joint that ros2_control should
manage, with one `command_interface` (either `position` or `velocity`) and
three `state_interface`s (`position`, `velocity` and `effort`):

```xml
<ros2_control name="diffbot" type="system">
  <hardware>...</hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-1</param>
      <param name="max"> 1</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

You can also give a joint an initial position:

```xml
<state_interface name="position">
  <param name="initial_value">1.57</param>
</state_interface>
```

**How isaac_ros2_utils handles this:** for each joint listed here,
`robot_controller.py` finds the corresponding joint in the imported USD stage
and configures its *drive*:

- Joints with a `position` command interface get a position drive. If the URDF
  does not specify a stiffness (see below), a large default stiffness
  (`1e8`) is set so the joint tracks position commands rigidly.
- Joints with a `velocity` command interface get a velocity drive. If the URDF
  does not specify a damping, a default damping (`15000`) is set.
- `initial_value` is written to the drive target at spawn time, so the robot
  starts in the given pose.

## Set up Joint Stiffness, Damping and Joint Friction

In Isaac Sim, a joint drive outputs the following force (this is how both
position control and velocity control are realized — with a high `stiffness`
the joint acts as a position servo, with `stiffness=0` and a high `damping`
it acts as a velocity servo):

```
force = stiffness * (target_position - position) + damping * (target_velocity - velocity)
```

Stiffness, damping and joint friction cannot be expressed in standard URDF.
This package defines them in an `isaac_drive_api` tag placed **inside the
normal `joint` tag** (not the one inside `ros2_control`):

```xml
<robot>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <axis xyz="0 0 1"/>
    <isaac_drive_api stiffness="0" damping="30000" joint_friction="10000"/>
  </joint>
</robot>
```

**How isaac_ros2_utils handles this:** `spawn.py` reads the tag after the URDF
import and writes the values to the joint prim in USD:

- `stiffness` / `damping` → the joint's `PhysicsDriveAPI` attributes
- `joint_friction` → the joint's `PhysxJointAPI` friction attribute

Rough tuning guide:

- Velocity-controlled wheels: `stiffness="0"`, large `damping`
- Position-controlled arm joints: large `stiffness`, moderate `damping`
- If you omit the tag, the defaults described in the previous section are
  applied depending on the command interface.

## Set up Friction of Link

Unlike Gazebo, Isaac Sim ties friction coefficients to a *physics material*
rather than to a link. This package therefore extends the URDF `material` tag
(normally only used for color) with an `isaac_rigid_body` tag:

```xml
<robot>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
    <isaac_rigid_body static_friction="1.0" dynamic_friction="1.0" restitution="0.0"/>
  </material>
</robot>
```

Assign the material to a link with the `visual` tag as usual (there is no need
to duplicate the material in the `collision` tag):

```xml
<robot>
  <link name="body_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.24 0.18 0.06" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="0.24 0.178 0.06" />
      </geometry>
    </collision>
  </link>
</robot>
```

**How isaac_ros2_utils handles this:** for every `material` with an
`isaac_rigid_body` tag, `spawn.py` creates a USD physics material
(`/<robot>/Looks/material_<name>`) with the given friction/restitution
values. Then, for every link whose `visual` uses that material, the physics
material is bound to the link's collision geometry, so the friction acts on
contacts of that link.

## Collision approximation (convex decomposition)

Isaac Sim's GPU physics cannot collide raw triangle meshes directly; mesh
collisions are approximated. By default this package imports collision
meshes with the **convex decomposition** approximation (the mesh is split
into multiple convex pieces).

You can tune the maximum number of convex pieces per link with a
`convex_decomposition` tag inside the `collision` tag:

```xml
<collision>
  <geometry>
    <mesh filename="package://my_robot/meshes/body.stl"/>
  </geometry>
  <convex_decomposition max_convex_hulls="16"/>
</collision>
```

**How isaac_ros2_utils handles this:** after the import, `spawn.py` finds the
collision geometry of each link and applies
`PhysxConvexDecompositionCollisionAPI` with the given
`max_convex_hulls` (default: 32). More hulls follow concave shapes more
accurately at the cost of performance.
