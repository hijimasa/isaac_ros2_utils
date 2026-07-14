# URDFへのセンサ情報の記述方法

このドキュメントでは、Isaac Sim上で動作するセンサ（および簡単なアクチュエータ）を
ロボットのURDFに記述する方法と、その記述がisaac_ros2_utilsによって
どのようにIsaac Sim上のオブジェクトへ変換されるかを解説します。

Gazebo ClassicではURDFの`<gazebo>`タグにセンサ情報を記述しますが、
このリポジトリでは`<robot>`タグ直下に置いた`<isaac>`タグで同じことを実現します。

```xml
<robot name="diffbot">
  ... リンクとジョイント ...
  <isaac>
    <sensor name="lidar_link" type="lidar"> ... </sensor>
    <sensor name="camera_link" type="camera"> ... </sensor>
    <surface_gripper name="body_link"> ... </surface_gripper>
    <thruster name="thruster_link"/>
  </isaac>
</robot>
```

`name`属性は常に、センサ／アクチュエータを取り付ける**リンク名**を指します。
ロボットのスポーン時に、`launch_sensor.py`（センサ）と`robot_controller.py`
（グリッパー・スラスター）がIsaac Simのステージ上からそのリンクのプリム
（＝USD上のオブジェクト）を探し出し、そこへセンサを取り付けます。

> **トピック名について:** 生成されるROS 2トピックには、リンクのプリムパスが
> プレフィックスとして付きます。例えばロボット`diffbot`の`lidar_link`に
> `<topic>scan</topic>`を指定すると、トピック名は
> `/diffbot/Geometry/base_link/lidar_link/scan`になります
> （Isaac Sim 6のURDFインポーターはリンクを`/<ロボット名>/Geometry/`以下に
> 配置します）。

> **空のリンクでも大丈夫:** センサ用リンクはvisual/collisionを持たないことが
> 多いですが、Isaac Sim 6のインポーターはそのようなリンクを生成しません。
> そのため`launch_sensor.py`が、URDFのジョイントで指定された位置姿勢に
> Xform（座標フレーム）を自動で作成します。

## LiDAR

```xml
<sensor name="lidar_link" type="lidar">
  <topic>scan</topic>
  <sensor_dimension_num>3</sensor_dimension_num> <!-- 2 or 3 -->
  <config>Example_Rotary</config>
</sensor>
```

| タグ | 意味 |
|----|----|
| `topic` | トピック名（リンクのプリムパスがプレフィックスに付きます） |
| `sensor_dimension_num` | `2`: `sensor_msgs/LaserScan`を配信 / `3`: `sensor_msgs/PointCloud2`を配信 |
| `config` | LiDARのセンサモデル名（下記参照） |

**isaac_ros2_utilsの内部処理:** リンクの下にRTX LiDARセンサ（Isaac SimのGPU
レイトレーシングによるLiDAR）を生成し、`ROS2RtxLidarHelper`ノードを持つ
OmniGraphが物理ステップごとにスキャンを配信します。

### `config`について（Isaac Sim 6）

Isaac Sim 6では従来のJSON形式のLiDARプロファイルが廃止されました。LiDARは
Isaac Simに同梱されたUSDセンサアセットとなり、**名前**で選択します。
指定できる名前には次のようなものがあります。

- 汎用サンプル: `Example_Rotary`（3D）、`Example_Rotary_2D`（2D）、
  `Example_Solid_State`
- 実在センサ: `SICK_TIM781`、`SICK_microScan3`、`RPLIDAR_S2E`、
  `OS0` / `OS1` / `OS2`（Ouster）、`HESAI_XT32_SD10`、`ZVISION_ML30S` など

`config`に未対応の名前（旧Hokuyo設定など）を指定した場合、isaac_ros2_utilsは
**対応している全ての名前を列挙した警告**を出力したうえで、
`sensor_dimension_num`が2なら`Example_Rotary_2D`、それ以外なら`Example_Rotary`
へ自動的にフォールバックします。

> **LiDARトピックにデータが来ない場合:** トピックは存在するのに何も配信されず、
> Isaac Simのログに`CUDA Driver CALL FAILED ... PTX ... unsupported toolchain`
> と出ている場合は、ホストのNVIDIAドライバーがIsaac Sim 6に対して古すぎます。
> ドライバーを575以降へ更新してください。

## カメラ

```xml
<sensor name="camera_link" type="camera">
  <topic>image_raw</topic>
  <horizontal_fov_rad>1.3962634</horizontal_fov_rad>
  <horizontal_focal_length>30</horizontal_focal_length> <!-- 光学パラメータ -->
  <vertical_focal_length>30</vertical_focal_length> <!-- 光学パラメータ -->
  <focus_distance>400</focus_distance> <!-- ピントが合う距離 -->
  <projection>perspective</projection> <!-- perspective または orthgonal -->
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

**isaac_ros2_utilsの内部処理:** リンクの下にUSDの`Camera`プリムを生成します
（ROSの慣習に合わせて、カメラがリンクの+X軸方向を向くよう回転されます）。
上記タグからアパーチャ・焦点距離・フォーカス距離・クリッピング範囲などの
光学属性が計算されます。`IsaacCreateRenderProduct`＋`ROS2CameraHelper`を持つ
OmniGraphがカメラ画像をレンダリングし、`sensor_msgs/Image`として配信します。

カメラのグラフは*再生ティック*で動作するため、画像はシミュレーション再生中
のみ配信されます。

## デプスカメラ

パラメータはカメラと同じで、`type`のみ異なります。

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

配信される画像は32bit floatのデプス画像です（`ROS2CameraHelper`の
`type: depth`）。

> **ヒント:** RViz2でデプス画像が真っ黒に見える場合は、画像表示設定の
> "Normalize Image"のチェックを外してください。

## 接触センサ

```xml
<sensor name="bumper_link" type="contact">
  <topic>contact</topic>
</sensor>
```

リンクが何かに接触している間`true`となる`std_msgs/Bool`を
`<リンクのプリムパス>/<topic>`へ配信します。

**isaac_ros2_utilsの内部処理:** リンクに接触レポート
（`PhysxContactReportAPI`）を有効化し、リンクのコリジョン形状の下に
Isaac Simの接触センサプリムを生成します。`IsaacReadContactSensor`＋
`ROS2Publisher`を持つOmniGraphが物理ステップごとに接触状態を配信します。

> **注意:** 接触センサはコリジョン形状を必要とします。リンクに`collision`
> タグが必要です。

## 吸着式グリッパー（surface gripper）

surface gripperは、吸着パッドのようにグリッパー面へ近づいた物体を掴む
機構です。`<isaac>`タグ直下に記述します。

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

| タグ | 意味 |
|----|----|
| `offset_x/y/z` | リンク座標系におけるグリッパー面の位置 [m] |
| `axis` | リンク座標系における吸着方向（`±1 0 0`、`0 ±1 0`、`0 0 ±1`のいずれか） |
| `grip_threshold` | 物体を掴める最大距離 [m] |
| `force_limit` | 吸着方向の最大保持力 [N] |
| `torque_limit` | 最大保持（せん断）トルク [N·m] |
| `retry_close` | `True`: 物体を掴めるまで吸着を試行し続ける |
| `bend_angle`、`stiffness`、`damping` | 旧バージョンとの互換用（Isaac Sim 6では現在使用されません） |

グリッパーは`<リンクのプリムパス>/toggle`へ`std_msgs/Bool`を配信して
操作します（`true`＝閉じる／吸着、`false`＝開く／解放）。

**isaac_ros2_utilsの内部処理:** Isaac Sim 6ではIsaac Simのロボットスキーマを
使って構築されます。`robot_controller.py`が`IsaacSurfaceGripper`プリムと
*アタッチメントポイント*（指定オフセットの位置で`axis`方向を向く、リンクに
固定されたD6ジョイント）を生成します。グリッパーを閉じると、Isaac Simが
`axis`方向`grip_threshold`以内の剛体を探し、ジョイントを介してリンクに
固定します。保持力が`force_limit`/`torque_limit`を超えると物体は外れます。
OmniGraphが`toggle`トピックを購読し、グリッパーの開閉APIを呼び出します。

## スラスター

スラスターはリンクの+Z軸方向へ推力を与える機構です。宇宙ロボットや
水中ロボットに使えます。

```xml
<thruster name="thruster_link"/>
```

推力[N]を`std_msgs/Float64`として`<リンクのプリムパス>/force`へ配信して
ください。値が0でない間、物理ステップごとに、リンクの位置に、リンクの
+Z軸方向の力が加わります（負の値で逆方向）。

**isaac_ros2_utilsの内部処理:** OmniGraphが`force`トピックを購読し、
PhysXの力適用APIでスラスターリンクのワールド姿勢に力を加えます。
スラスターリンク自体が形状を持たない（＝剛体でない）場合は、最も近い
剛体の祖先リンクに対して、スラスターリンクの位置で力を加えるため、
モーメントアームは正しく考慮されます。
