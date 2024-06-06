# Set up URDF for ros2_control

## Introduction to ros2_control

The ros2_control offers a developers a common API that allows your software to switch between many different robot types, and the sensors they have built in, by simply changing some launch arguments. For example if we inspect the Panda Robot’s ros2_control.xacro we can see it uses a flag use_fake_hardware to switch between being simulated or connecting to a physical robot.

```xml
<ros2_control>
  <hardware>
    <xacro:if value="${use_fake_hardware}">
      <plugin>mock_components/GenericSystem</plugin>
    </xacro:if>
    <xacro:unless value="${use_fake_hardware}">
      <plugin>franka_hardware/FrankaHardwareInterface</plugin>
      <param name="robot_ip">${robot_ip}</param>
    </xacro:unless>
  </hardware>
</ros2_control>
```

Hardware Components can be of different types, but the plugin "mock_components/GenericSystem" is very a simple System that forwards the incoming command_interface values to the tracked state_interface of the joints (i.e., perfect control of the simulated joints).

To use ros2_control with Isaac Sim, we have to introduce isaac_ros2_control. This Hardware Interface is a System that read / write joint state / command from shared memory.
There is another method using topic_based_ros2_control, but this one did not work well with mobile robots due to problems with positional commands.
isaac_ros2_control retrieves the shared memory associated with the name attribute of the robot tag and reads and writes information about the joint.
Therefore, when operating multiple robots, it is necessary to change the name attribute value of the robot tag of the URDF to be read.
The following is an example of introducing isaac_ros2_control.

```xml
<ros2_control>
  <hardware>
    <xacro:if value="${use_fake_hardware}">
      <plugin>fake_components/GenericSystem</plugin>
    </xacro:if>
    <xacro:if value="${use_sim}">
      <plugin>isaac_ros2_control/IsaacSystem</plugin>
    </xacro:if>
  </hardware>
</ros2_control>
```

## Set up Joint Information

Within the ros2_control tag in the URDF, joint information must be included in addition to the ros2_control plugin information.
The following is an example of description of joint information.

```xml
<ros2_control>
  <joint>
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

Note that the joint information used in isaac_ros2_control must contain one command_interface (position or velocity) and three state_interface (position and velocity and effort).

## Set up Joint Stiffness, Damping and Joint Friction

In Isaac Sim, the force output by the joint is given by
```
force=stiffness*(position－targetposition)+damping*(velocity－targetvelocity)
```

The stiffness, damping and joint friction are not defined in default URDF.
In this package, they are defined in isaac_drive_api tag in joint tag like below.

```
<robot>
  <joint>
    <isaac_drive_api stiffness="0" damping="150000" joint_friction="1000"/>
  </joint>
</robot>
```

Note that the joint tag that include the isaac_drive_api tag is not exist in ros2_control tag.

In default behaviour, the stiffness and damping are set zero.

## Set up Friction of Link

Unlike Gazebo, Isaac Sim allows the user to set the static and dynamic friction coefficients.
Since these parameters are tied to the material, URDF adds friction information to the material tag, which is normally used to set the color.
The following is an example of a description of a material with friction set:

```
<robot>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
    <isaac_rigid_body static_friction="1.0" dynamic_friction="1.0"/>
  </material>
</robot>
```

As with regular URDF, you can set friction on the link by setting the material in the visual tag.
Example:

```
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

Note that there is no need to duplicate the material in the collision tag.
