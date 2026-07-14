# URDFへのros2_controlタグの記述方法

このドキュメントでは、Isaac Sim上のロボットを
[ros2_control](https://control.ros.org/)で制御するためのURDFの書き方と、
その記述をisaac_ros2_utilsが内部でどのように処理しているかを解説します。

## ros2_controlとは

ros2_controlは、URDF内のプラグイン指定を変えるだけで、実機・シミュレーターなど
さまざまな「ハードウェア」を同じコントローラー（`diff_drive_controller`や
`joint_trajectory_controller`など）で扱えるようにする共通APIです。
コントローラー側は一切変更せず、実際のロボットやシミュレーターと通信する
*ハードウェアインターフェース*だけを差し替えます。

Isaac Simでros2_controlを使うために、このリポジトリでは
[topic_based_ros2_control](https://github.com/hijimasa/topic_based_ros2_control)
を使用します。このハードウェアインターフェースは、2つのROS 2トピックを介して
`sensor_msgs/JointState`メッセージをシミュレーターとやり取りします。

- 関節状態トピックを**購読**します。Isaac Simが各関節の実際の位置・速度・トルクを
  そこへ配信します。
- 関節指令トピックへ**配信**します。Isaac Simがそれを購読し、関節のドライブへ
  指令を渡します。

トピック名はhardwareパラメータとして宣言します。

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

**isaac_ros2_utilsの内部処理:** ロボットのスポーン時に`robot_controller.py`が
URDFから`joint_commands_topic`と`joint_states_topic`を読み取り、Isaac Sim内に
OmniGraph（物理ステップごとに評価されるノードネットワーク）を構築します。

```
OnPhysicsStep ─┬─> ArticulationState ──> ROS2Publisher  ──> joint_states_topic
               └─> ROS2Subscriber(joint_commands_topic) ──> ArticulationController
```

これにより、ROS 2側から見るとIsaac Simは「関節状態を報告し、関節指令を受け取る
ハードウェア」として振る舞います。これは`TopicBasedSystem`が期待する形そのものです。

使用できるhardwareパラメータ:

| パラメータ | 意味 |
|----|----|
| `joint_commands_topic` | Isaac Simが関節指令を受け取るトピック |
| `joint_states_topic` | Isaac Simが関節状態を配信するトピック |
| `sum_wrapped_joint_states` | 車輪などcontinuousジョイントを持つロボットでは`true`にしてください。Isaac Simは±2πに折り返された角度を報告するため、このオプションでハードウェアインターフェース側が連続的な角度に積算します。`diff_drive_controller`のオドメトリ計算に必須です。 |
| `trigger_joint_command_threshold` | 指令値がこの値より大きく変化したときだけ指令メッセージを配信します（省略可）。 |

> **補足:** トピック名はロボット名とは独立です。複数台のロボットを動かす場合は、
> ロボットごとに別のトピック名（例: ロボット名をプレフィックスにする）を
> 指定してください。

## 関節情報の記述

`ros2_control`タグの中に、ros2_controlで管理する各関節を記述します。
`command_interface`は1つ（`position`または`velocity`）、`state_interface`は
3つ（`position`・`velocity`・`effort`）が必要です。

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

関節の初期位置を与えることもできます。

```xml
<state_interface name="position">
  <param name="initial_value">1.57</param>
</state_interface>
```

**isaac_ros2_utilsの内部処理:** ここに記述された各関節について、
`robot_controller.py`がインポート後のUSDステージから対応するジョイントを探し、
その*ドライブ*を設定します。

- `position`のcommand_interfaceを持つ関節は位置ドライブになります。URDFで
  スティフネスが指定されていない場合、位置指令に追従するよう大きな既定値
  （`1e8`）が設定されます。
- `velocity`のcommand_interfaceを持つ関節は速度ドライブになります。URDFで
  ダンピングが指定されていない場合、既定値（`15000`）が設定されます。
- `initial_value`はスポーン時にドライブの目標値へ書き込まれ、ロボットは
  指定した姿勢で開始します。

## 関節のスティフネス・ダンピング・摩擦の設定

Isaac Simでは、関節のドライブは次の式で力を出力します（位置制御も速度制御も
この式で実現されます。`stiffness`を大きくすると位置サーボ、`stiffness=0`で
`damping`を大きくすると速度サーボとして働きます）。

```
force = stiffness * (target_position - position) + damping * (target_velocity - velocity)
```

スティフネス・ダンピング・関節摩擦は標準のURDFでは記述できないため、
このパッケージでは通常の`joint`タグの中（`ros2_control`タグの中の`joint`では
ありません）に`isaac_drive_api`タグを置いて記述します。

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

**isaac_ros2_utilsの内部処理:** URDFインポート後に`spawn.py`がこのタグを読み、
USD上のジョイントプリムへ値を書き込みます。

- `stiffness` / `damping` → ジョイントの`PhysicsDriveAPI`属性
- `joint_friction` → ジョイントの`PhysxJointAPI`の摩擦属性

おおまかな調整の指針:

- 速度制御の車輪: `stiffness="0"`、大きめの`damping`
- 位置制御のアーム関節: 大きめの`stiffness`、適度な`damping`
- タグを省略した場合は、前節で説明したcommand_interfaceに応じた既定値が
  適用されます。

## リンクの摩擦の設定

GazeboではリンクごとにURDFで摩擦を設定しますが、Isaac Simでは摩擦係数は
*物理マテリアル*に紐づきます。そこでこのパッケージでは、本来は色の設定にしか
使われないURDFの`material`タグを`isaac_rigid_body`タグで拡張しています。

```xml
<robot>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
    <isaac_rigid_body static_friction="1.0" dynamic_friction="1.0" restitution="0.0"/>
  </material>
</robot>
```

通常のURDFと同様に、`visual`タグでマテリアルを指定すればそのリンクに摩擦が
設定されます（`collision`タグにマテリアルを重複して書く必要はありません）。

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

**isaac_ros2_utilsの内部処理:** `isaac_rigid_body`タグを持つ各`material`に
ついて、`spawn.py`が指定された摩擦・反発係数を持つUSD物理マテリアル
（`/<ロボット名>/Looks/material_<名前>`）を作成します。そして、そのマテリアルを
`visual`で使用している各リンクのコリジョン形状へ物理マテリアルをバインドし、
そのリンクの接触に摩擦が効くようにします。

## コリジョンの近似（凸分解）

Isaac SimのGPU物理演算は三角形メッシュ同士をそのまま衝突させることができず、
メッシュのコリジョンは近似されます。このパッケージでは既定で
**凸分解（convex decomposition）**近似（メッシュを複数の凸形状に分割）で
インポートします。

リンクごとの凸形状の最大数は、`collision`タグ内の`convex_decomposition`タグで
調整できます。

```xml
<collision>
  <geometry>
    <mesh filename="package://my_robot/meshes/body.stl"/>
  </geometry>
  <convex_decomposition max_convex_hulls="16"/>
</collision>
```

**isaac_ros2_utilsの内部処理:** インポート後に`spawn.py`が各リンクの
コリジョン形状を探し、指定された`max_convex_hulls`（既定値: 32）で
`PhysxConvexDecompositionCollisionAPI`を適用します。凸形状の数を増やすほど
凹んだ形状を正確に再現できますが、計算負荷が上がります。
