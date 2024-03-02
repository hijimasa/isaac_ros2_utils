# Set up URDF for ros2_control

## Introduction to ros2_control

The ros2_control offers a developers a common API that allows your software to switch between many different robot types, and the sensors they have built in, by simply changing some launch arguments. For example if we inspect the Panda Robot’s ros2_control.xacro we can see it uses a flag use_fake_hardware to switch between being simulated or connecting to a physical robot.

```xml
<hardware>
  <xacro:if value="${use_fake_hardware}">
    <plugin>mock_components/GenericSystem</plugin>
  </xacro:if>
  <xacro:unless value="${use_fake_hardware}">
    <plugin>franka_hardware/FrankaHardwareInterface</plugin>
    <param name="robot_ip">${robot_ip}</param>
  </xacro:unless>
</hardware>
```

Hardware Components can be of different types, but the plugin "mock_components/GenericSystem" is very a simple System that forwards the incoming command_interface values to the tracked state_interface of the joints (i.e., perfect control of the simulated joints).

To use ros2_control with Isaac Sim, we have to introduce isaac_ros2_control. This Hardware Interface is a System that read / write joint state / command from shared memory.
There is another method using topic_based_ros2_control, but this one did not work well with mobile robots due to problems with positional commands.
isaac_ros2_control retrieves the shared memory associated with the name attribute of the robot tag and reads and writes information about the joint.
Therefore, when operating multiple robots, it is necessary to change the name attribute value of the robot tag of the URDF to be read.
The following is an example of introducing isaac_ros2_control.

```xml
<hardware>
  <xacro:if value="${use_fake_hardware}">
    <plugin>fake_components/GenericSystem</plugin>
  </xacro:if>
  <xacro:if value="${use_sim}">
    <plugin>isaac_ros2_control/IsaacSystem</plugin>
  </xacro:if>
</hardware>
```

## Set up Joint Information

Within the ros2_control tag in the URDF, joint information must be included in addition to the ros2_control plugin information.
The following is an example of description of joint information.

```xml
<joint>
  <command_interface name="velocity">
    <param name="min">-1</param>
    <param name="max"> 1</param>
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

Note that the joint information used in isaac_ros2_control must contain one command_interface (position or velocity) and three state_interface (position and velocity and effort).