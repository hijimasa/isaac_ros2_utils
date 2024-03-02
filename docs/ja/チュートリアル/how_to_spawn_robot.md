# ロボットの生成方法

このドキュメントでは、シミュレータ上にロボットを生成する手順について説明します。
ロボットモデルの生成は、大きく分けて以下の３つの手順に分けられます。

1. ロボットの物理モデルの生成
2. ロボットのセンサモデルの生成
3. ロボットコントローラの生成

以下では３つの手順の詳細を説明します。

## ロボットの物理モデルの生成

ロボットの物理モデルを生成するノードを作成する例を以下に示します。

```python
    isaac_diffbot_description_path = os.path.join(
        get_package_share_directory('diffbot_description'))

    xacro_file = os.path.join(isaac_diffbot_description_path,
                              'robots',
                              'diffbot.urdf.xacro')
    urdf_path = os.path.join(isaac_diffbot_description_path, 'robots', 'diffbot.urdf')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
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

上記の例にあるように、物理モデルの生成には、isaac_ros2_scriptsパッケージのspawn_robotを使用します。
spawn_robotは引数として、生成するロボットのURDFファイル、位置（X,Y,Z）と姿勢(Roll,Pitch,Yaw)とロボットが固定されているかどうかを与える必要があります。
ロボットが固定されているかどうかの引数は、アームロボットが環境に固定されている場合に使用されます。

## ロボットのセンサモデルの生成

ロボットのセンサモデルを生成するノードを作成する例を以下に示します。

```python
    isaac_prepare_sensors = Node(
        package="isaac_ros2_scripts",
        executable="prepare_sensors",
        parameters=[{'urdf_path': str(urdf_path)}],
    )
```

ロボットのセンサモデルの生成には、URDFファイル内のisaacタグ内の情報が必要なので、prepare_sensorsは引数として生成するロボットのURDFファイルが必要です。


## ロボットコントローラの生成

ロボットのコントローラを生成するノードを作成する例を以下に示します。
ここでいうコントローラは、ros2_controllerではなく共有メモリの値をもとにシミュレータ上のロボットを駆動させる内部コントローラを指します。

```python
    isaac_prepare_robot_controller = Node(
        package="isaac_ros2_scripts",
        executable="prepare_robot_controller",
        parameters=[{'urdf_path': str(urdf_path)}],
    )
```

ロボットのコントローラの生成には、URDFファイル内のros2_controlタグ内の情報が必要なので、prepare_robot_controllerには引数として生成するロボットのURDFファイルが必要です。

## ノードの立ち上げ方について

一度起動しているIsaac Simに追加でロボットモデルを生成する方法として、このリポジトリではIsaac SimのPython REPLエクステンションを利用しています。
このエクステンションは同時に複数の処理を与えることが出来ないので、ノードを起動する際にはRegisterEventHandlerを使用して順番にノードを処理するようにしてください。

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
