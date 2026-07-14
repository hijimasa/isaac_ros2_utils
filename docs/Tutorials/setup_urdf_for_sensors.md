# Set up URDF for Sensors

This document shows how to describe sensors (and simple actuators) that run
on Isaac Sim in the robot's URDF, and how isaac_ros2_utils turns each
description into Isaac Sim objects.

In Gazebo Classic, sensors are described in `<gazebo>` tags in the URDF. This
repository achieves the same with an `<isaac>` tag placed directly under the
`<robot>` tag:

```xml
<robot name="diffbot">
  ... links and joints ...
  <isaac>
    <sensor name="lidar_link" type="lidar"> ... </sensor>
    <sensor name="camera_link" type="camera"> ... </sensor>
    <surface_gripper name="body_link"> ... </surface_gripper>
    <thruster name="thruster_link"/>
  </isaac>
</robot>
```

The `name` attribute always refers to the **link** the sensor/actuator is
attached to. When the robot is spawned, `launch_sensor.py` (sensors) and
`robot_controller.py` (gripper/thruster) look up the prim (= the USD object)
of that link on the Isaac Sim stage and attach the sensor to it.

> **Topic names:** the generated ROS 2 topics are prefixed with the prim path
> of the link. For example, with the robot `diffbot` and
> `<topic>scan</topic>` on `lidar_link`, the topic becomes
> `/diffbot/Geometry/base_link/lidar_link/scan`.
> (Isaac Sim 6's URDF importer places links under `/<robot>/Geometry/`.)

> **Empty links are fine:** a sensor link often has no visual/collision
> geometry. Isaac Sim 6 does not create such links during import, so
> `launch_sensor.py` creates an Xform (a coordinate frame) for them at the
> pose given by the joint in the URDF.

## LiDAR

```xml
<sensor name="lidar_link" type="lidar">
  <topic>scan</topic>
  <sensor_dimension_num>3</sensor_dimension_num> <!-- 2 or 3 -->
  <config>Example_Rotary</config>
</sensor>
```

| Tag | Meaning |
|----|----|
| `topic` | Topic name (prefixed with the link's prim path) |
| `sensor_dimension_num` | `2`: publish `sensor_msgs/LaserScan` / `3`: publish `sensor_msgs/PointCloud2` |
| `config` | Name of the LiDAR sensor model (see below) |

**How isaac_ros2_utils handles this:** an RTX LiDAR sensor (Isaac Sim's
GPU-raytraced lidar) is created under the link, and an OmniGraph with a
`ROS2RtxLidarHelper` node publishes the scan on every physics step.

### About `config` (Isaac Sim 6)

Isaac Sim 6 removed the old JSON lidar profiles. LiDARs are now USD sensor
assets bundled with Isaac Sim and are selected **by name**. Available names
include:

- Generic examples: `Example_Rotary` (3D), `Example_Rotary_2D` (2D),
  `Example_Solid_State`
- Real sensor models: `SICK_TIM781`, `SICK_microScan3`, `RPLIDAR_S2E`,
  `OS0` / `OS1` / `OS2` (Ouster), `HESAI_XT32_SD10`, `ZVISION_ML30S`, etc.

If the name in `config` is not supported (for example the old Hokuyo
configs), isaac_ros2_utils prints a warning **listing all supported names**
and automatically falls back to `Example_Rotary_2D` (when
`sensor_dimension_num` is 2) or `Example_Rotary` (otherwise).

> **No data on the LiDAR topic?** If the topic exists but nothing is
> published and the Isaac Sim log shows `CUDA Driver CALL FAILED ... PTX ...
> unsupported toolchain`, your host NVIDIA driver is too old for Isaac Sim 6.
> Upgrade to driver 575 or later.

## Camera

```xml
<sensor name="camera_link" type="camera">
  <topic>image_raw</topic>
  <horizontal_fov_rad>1.3962634</horizontal_fov_rad>
  <horizontal_focal_length>30</horizontal_focal_length> <!-- optical parameter -->
  <vertical_focal_length>30</vertical_focal_length> <!-- optical parameter -->
  <focus_distance>400</focus_distance> <!-- distance for clear image -->
  <projection>perspective</projection> <!-- perspective or orthgonal -->
  <image>
     <width>600</width>
     <height>600</height>
  </image>
  <clip>
    <near>0.02</near>
    <far>300</far>
  </clip>
  <update_rate>10.0</update_rate>
</sensor>
```

**How isaac_ros2_utils handles this:** a USD `Camera` prim is created under
the link (rotated so that the camera looks along the link's +X axis, the ROS
convention), and its optical attributes (aperture, focal length, focus
distance, clipping range) are computed from the tags above. An OmniGraph with
`IsaacCreateRenderProduct` + `ROS2CameraHelper` renders the camera image and
publishes it as `sensor_msgs/Image`.

The camera graph runs on the *playback tick*, so images are published only
while the simulation is playing.

## Depth Camera

Same parameters as the camera; only the `type` differs:

```xml
<sensor name="depth_camera_link" type="depth_camera">
  <topic>image_raw</topic>
  <horizontal_fov_rad>1.3962634</horizontal_fov_rad>
  <horizontal_focal_length>30</horizontal_focal_length>
  <vertical_focal_length>30</vertical_focal_length>
  <focus_distance>400</focus_distance>
  <projection>perspective</projection>
  <image>
    <width>320</width>
    <height>240</height>
  </image>
  <clip>
    <near>0.1</near>
    <far>100</far>
  </clip>
  <update_rate>10</update_rate>
</sensor>
```

The published image is a 32-bit float depth image (`type: depth` of
`ROS2CameraHelper`).

> **Tip:** if the depth image looks black in RViz2, uncheck
> "Normalize Image" in the image display settings.

## Contact Sensor

```xml
<sensor name="bumper_link" type="contact">
  <topic>contact</topic>
</sensor>
```

Publishes `std_msgs/Bool` (`true` while the link touches something) on
`<link prim path>/<topic>`.

**How isaac_ros2_utils handles this:** a contact report is enabled on the
link (`PhysxContactReportAPI`), an Isaac Sim contact sensor prim is created
under the link's collision geometry, and an OmniGraph with
`IsaacReadContactSensor` + `ROS2Publisher` publishes the contact state on
every physics step.

> **Note:** contact sensors need collision geometry — the link must have a
> `collision` tag.

## Surface Gripper (vacuum gripper)

A surface gripper grabs an object that comes close to the gripper surface,
like a vacuum pad. It is described directly under the `<isaac>` tag:

```xml
<surface_gripper name="body_link">
  <topic>gripper_enable</topic>
  <offset_x>1.0</offset_x>
  <offset_y>0</offset_y>
  <offset_z>0</offset_z>
  <axis>1 0 0</axis>
  <grip_threshold>0.1</grip_threshold>
  <force_limit>1.0e2</force_limit>
  <torque_limit>1.0e3</torque_limit>
  <bend_angle>0.7854</bend_angle>
  <stiffness>1.0e4</stiffness>
  <damping>1.0e3</damping>
  <retry_close>False</retry_close>
</surface_gripper>
```

| Tag | Meaning |
|----|----|
| `offset_x/y/z` | Position of the gripper surface, relative to the link frame [m] |
| `axis` | Gripping direction in the link frame (one of `±1 0 0`, `0 ±1 0`, `0 0 ±1`) |
| `grip_threshold` | Maximum distance at which an object can be grabbed [m] |
| `force_limit` | Maximum holding force along the gripping axis [N] |
| `torque_limit` | Maximum holding (shear) torque [N·m] |
| `retry_close` | `True`: keep trying to grip until an object is caught |
| `bend_angle`, `stiffness`, `damping` | Kept for compatibility with older Isaac Sim versions (currently not applied on Isaac Sim 6) |

Control the gripper by publishing `std_msgs/Bool` to
`<link prim path>/toggle` (`true` = close/grip, `false` = open/release).

**How isaac_ros2_utils handles this:** on Isaac Sim 6, the gripper is built
on Isaac Sim's robot schema. `robot_controller.py` creates an
`IsaacSurfaceGripper` prim and an *attachment point* (a D6 joint fixed on the
link at the given offset, pointing along `axis`). When the gripper closes,
Isaac Sim searches for a rigid body within `grip_threshold` along the axis
and locks it to the link through the joint; force/torque limits decide when
the object is dropped. An OmniGraph subscribes to the `toggle` topic and
calls the gripper open/close API.

## Thruster

A thruster applies a pushing force along the link's +Z axis — useful for
free-flying robots or underwater robots:

```xml
<thruster name="thruster_link"/>
```

Publish the desired force [N] as `std_msgs/Float64` to
`<link prim path>/force`. The force is applied at the link's position, along
the link's +Z axis (a negative value pushes backward), on every physics step
while the value is non-zero.

**How isaac_ros2_utils handles this:** an OmniGraph subscribes to the `force`
topic and applies the force through PhysX's force API at the thruster link's
world pose. If the thruster link itself has no geometry (and therefore no
rigid body), the force is applied to the nearest ancestor link that is a
rigid body, still at the thruster link's pose — so lever arms are respected.
