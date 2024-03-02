# Set up URDF for Sensors

This document shows how to describe the information of a sensor running on Isaac Sim in the robot's URDF.

In Gazebo Classic, it was possible to generate sensors in the simulation by writing sensor information in the gazebo tag of the URDF.
In this repository, the same functionality is achieved by describing sensor information in isaac tags.
Below we show how to describe sensor information for LiDAR, cameras and depth cameras.

## Description of LiDAR information

The following is an example of LiDAR information.

```xml
<sensor name="lidar_link" type="lidar">
  <topic>scan</topic>
  <sensor_dimension_num>3</sensor_dimension_num> <!-- 2 or 3 -->
  <config>Example_Rotary</config>
  <!-- Config Example
  <config>Example_Rotary</config>
  <config>slamtec/RPLIDAR_S2E</config>
  <config>hokuyo/UST-30LX</config>
  -->
</sensor>
```

As shown in the example above, the basic description is the same as for the gazebo tag, but you can see that the setting values have changed.
sensor_dimension_num is the dimension of the point cloud to be acquired, and can be set to 2D or 3D.
config can be a LiDAR configuration file.
The LiDAR configuration file is located in "exts/omni.isaac.sensor/data/lidar_configs" in "exts/omni.isaac.sensor/data/lidar_configs" in the Isaac Sim reference directory ("~/.local/share/ov/pkg/isaac_sim-2023.1.1/" for native or "/isaac-sim" for Docker).
Please add configuration files as needed.

## Description of Camera information

The following is an example of camera information.

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

## Description of Depth Camera information

The following is an example of depth camera information.

```xml
<sensor name="depth_camera_link" type="depth_camera">
  <topic>image_raw</topic>
  <horizontal_fov_rad>1.3962634</horizontal_fov_rad>
  <horizontal_focal_length>30</horizontal_focal_length> <!-- optical parameter -->
  <vertical_focal_length>30</vertical_focal_length> <!-- optical parameter -->
  <focus_distance>400</focus_distance> <!-- distance for clear image -->
  <projection>perspective</projection> <!-- perspective or orthgonal -->
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