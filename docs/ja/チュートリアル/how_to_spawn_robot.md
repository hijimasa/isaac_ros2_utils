# ロボットの生成方法

このドキュメントでは、起動中のシミュレーターにロボットを配置する方法を
説明します。

スポーンは`spawn_robot`ノードで行います。このノードは、事前に起動した
シミュレーター（[シミュレーションランチャーの使い方](./how_to_use_simulator_launcher.md)参照）
のREST APIへURDFを送信し、シミュレーター側が1回のリクエストで次の3ステップを
実行します。

1. **物理モデルの生成** — Isaac SimのURDFインポーターでURDFをUSDへ変換し、
   指定した位置姿勢でステージへ配置します。`isaac_drive_api`・
   `isaac_rigid_body`・`convex_decomposition`タグはここで適用されます
   （[URDFへのros2_controlタグの記述方法](./setup_urdf_for_ros2_control.md)参照）。
2. **ロボットコントローラーの生成** — `ros2_control`タグから、ros2_controlと
   `sensor_msgs/JointState`をやり取りするOmniGraphを生成します。`isaac`タグの
   グリッパーやスラスターもここで生成されます。
3. **センサの生成** — `isaac`タグのLiDAR・カメラ・接触センサを、配信用の
   グラフとともに生成します
   （[URDFへのセンサ情報の記述方法](./setup_urdf_for_sensors.md)参照）。

## launchファイルからのスポーン

ロボットの記述は通常xacroファイルで書かれているため、まずURDFファイルへ
展開してから、そのパスを`spawn_robot`へ渡します。

```python
    import xacro

    isaac_diffbot_description_path = os.path.join(
        get_package_share_directory('diffbot_description'))

    xacro_file = os.path.join(isaac_diffbot_description_path,
                              'robots',
                              'diffbot.urdf.xacro')
    urdf_path = os.path.join(isaac_diffbot_description_path, 'robots', 'diffbot.urdf')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

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

`spawn_robot`のパラメータ:

| パラメータ | 意味 |
|----|----|
| `urdf_path` | URDFファイルのパス（必須） |
| `x`、`y`、`z` | スポーン位置 [m] |
| `R`、`P`、`Y` | スポーン姿勢（ロール／ピッチ／ヨー） [rad] |
| `fixed` | `True`: ロボットの基部をワールドに固定します（環境に固定されたアームロボット向け） |
| `api_host`、`api_port` | シミュレーターのREST APIのアドレス（デフォルト: `localhost:8080`） |

ノードはスポーン完了後に終了するため、必要なら`RegisterEventHandler`＋
`OnProcessExit`で後続の処理を繋げられます。1回のリクエストで3ステップ全てが
実行されるので、特別な起動順序の制御は不要です。ros2_controlのノード群と
同じlaunchファイルに`spawn_robot`を入れるだけで動きます（完全な例は
[移動ロボットのデモ](../デモ/demo_for_mobile_robot.md)を参照してください）。

## 物体（USDアセット）のシーンへの追加

ロボット以外にも、任意のUSDアセット（小物・棚・ワークなど）を`add_usd`
ノードで追加できます。

```python
    isaac_add_usd = Node(
        package="isaac_ros2_scripts",
        executable="add_usd",
        parameters=[{'usd_path': str(usd_path),
                    'usd_name': 'target_object',
                    'x' : 1.0,
                    'y' : 0.0,
                    'z' : 0.0,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 0.0,
                    }],
    )
```

| パラメータ | 意味 |
|----|----|
| `usd_path` | USDファイルのパス（必須） |
| `usd_name` | ステージ上での物体の名前。`/World/<usd_name>`に配置されます |
| `x` ～ `Y` | 物体の位置姿勢 |
| `api_host`、`api_port` | シミュレーターのREST APIのアドレス |

## 追加した物体のTF配信

追加した物体の位置を知りたい場合（例: 把持対象の位置）、`publish_tf`ノードで
物体のリンクのTF配信をセットアップできます。

```python
    isaac_publish_tf = Node(
        package="isaac_ros2_scripts",
        executable="publish_tf",
        parameters=[{'robot_name': 'target_object',
                    'target_link': 'body_link',
                    }],
    )
```

| パラメータ | 意味 |
|----|----|
| `robot_name` | `add_usd`で指定した`usd_name` |
| `target_link` | TFとして配信したいリンク（プリム）の名前 |
| `api_host`、`api_port` | シミュレーターのREST APIのアドレス |

位置姿勢は物理ステップごとに`/tf`へ配信されます。
