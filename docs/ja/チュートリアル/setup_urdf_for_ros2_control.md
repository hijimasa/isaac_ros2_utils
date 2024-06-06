# URDFへのros2_cotnrolタグの記述方法

このドキュメントではIsaac Sim上でros2_controlで制御できるロボットのURDFを記述する方法を示します。

## ros2_controlの導入

ros2_controlは開発者に共通のAPIを提供し、起動時の引数を変更するだけで、様々なタイプのロボットや内蔵センサを切り替えることができます。例えば、pandaのros2_control.xacroを見てみると、use_fake_hardwareというフラグを使って、シミュレーションか物理ロボットへの接続かを切り替えていることがわかります。

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

ハードウェア・コンポーネントには様々なタイプがありますが、プラグイン "mock_components/GenericSystem "は、入力されたcommand_interfaceの値を、関節の追跡されたstate_interfaceに転送する（つまり、シミュレートされた関節を完璧に制御する）非常にシンプルなシステムです。

Isaac Simでros2_controlを使用するには、isaac_ros2_controlを導入する必要があります。このハードウェアインターフェースは、共有メモリから関節の状態やコマンドを読み書きするシステムです。
他にもtopic_based_ros2_controlを使う方法もありますが、こちらは位置指令によって関節角度が固定される問題があり、移動ロボットではうまく動作しませんでした。
isaac_ros2_controlは、ロボットタグのname属性に関連付けられた共有メモリを取得し、関節に関する情報を読み書きします。
そのため、複数のロボットを操作する場合は、読み込むURDFのロボットタグのname属性値を変更する必要があります。
以下にisaac_ros2_controlの導入例を示します。

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

## ジョイント情報の設定

URDFのros2_controlタグ内には、使用するros2_controlプラグインの情報に加えて、ジョイントの情報を記載する必要があります。
以下はジョイント情報の設定の例です。

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

ここで、isaac_ros2_controlで使用するジョイント情報は、command_interfaceを一つ（位置か速度）とstate_interfaceを３つ（位置、速度、力）含んでいる必要があることに注意してください。

## ジョイントの剛性、ダンパ係数、関節摩擦の設定

Isaac Simでは、ジョイントが発生させる力は以下の式で与えられる。
```
force=stiffness*(position－targetposition)+damping*(velocity－targetvelocity)
```

剛性(stiffness)とダンパ係数(damping)、関節摩擦(joint friction)は通常のURDFでは記述できない。
本パッケージでは、以下の記述のようにjointタグ内にisaac_drive_apiというタグを用意し、その中に記述するようにしている。

```
<robot>
  <joint>
    <isaac_drive_api stiffness="0" damping="150000" joint_friction="1000"/>
  </joint>
</robot>
```

isaac_drive_apiタグが含まれるjointタグは、ros2_controlタグ内ではないことに気をつけてください。

isaac_drive_apiを含まないジョイントでは、剛性とダンパ係数は0に設定されます。

## リンクの摩擦の設定

Isaac SimではGazeboと異なり、静止摩擦係数と動摩擦係数を設定することができます。
これらのパラメータはマテリアルに紐付けられるため、URDFでは通常色を設定するために利用されていたmaterialタグに摩擦の情報を付与するようにしました。
以下が摩擦を設定したマテリアルの記述の例です。

```
<robot>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
    <isaac_rigid_body static_friction="1.0" dynamic_friction="1.0"/>
  </material>
</robot>
```

通常のURDFと同様に、visualタグにマテリアルを設定することで、リンクに摩擦を設定することができます。
例：

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

collisionタグに重複してマテリアルを記述しなくて良いことに気をつけてください。

