# URDFへのセンサ情報の記述方法

このドキュメントではIsaac Sim上で動作するセンサの情報をロボットのURDFに記述する方法を示します。

Gazebo ClassicではURDFのgazeboタグにセンサ情報を記載することで、シミュレーション上にセンサを生成することが出来ていました。
そこで、このリポジトリではisaacタグにセンサ情報を記載することで、同様の機能を実現しています。
以下では、LiDAR、カメラおよびデプスカメラのセンサ情報の記述方法について示します。

## LiDAR情報の記述

以下にLiDAR情報の例を示します。

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

上記の例にあるように、基本的な記述方法はgazeboタグの際と同じですが、設定値が変わっていることがわかります。
sensor_dimension_numは取得する点群の次元で、２次元または３次元が設定できます。
configには、LiDARの設定ファイルを指定できます。
LiDARの設定ファイルは、Isaac Simの基準ディレクトリ（ネイティブなら"~/.local/share/ov/pkg/isaac-sim_2023.1.1/"、Dockerなら"/isaac-sim"）内の"exts/omni.isaac.sensor/data/lidar_configs"にあります。
必要の応じて設定ファイルを追加して対応してください。

## カメラ情報の記述

以下にカメラ情報の例を示します。

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

## デプスカメラ情報の記述

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