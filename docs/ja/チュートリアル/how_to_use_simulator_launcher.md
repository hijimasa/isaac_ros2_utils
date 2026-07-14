# シミュレーションランチャーの使い方

Gazebo ClassicがROS 2のlaunchファイルからシミュレーターを起動できたのと
同じように、このリポジトリではシミュレーションランチャーノードでIsaac Simを
起動できます。

ランチャーはIsaac Simを起動し（初回は数分かかります）、ステージ
（環境を記述したUSDファイル）を読み込み、Isaac Sim内に小さな
**REST APIサーバー**を立ち上げます。ロボットや物体は後からこのAPI経由で
追加します（[ロボットの生成方法](./how_to_spawn_robot.md)参照）。

## コマンドラインからの使用

```bash
ros2 run isaac_ros2_scripts launcher
```

このコマンドは地面だけのデフォルト環境を読み込みます。

ログに"Simulation ready"と表示されたら、REST APIが
`http://localhost:8080`で利用可能になっています。Isaac Simウィンドウの
**Play**ボタン（または`POST /simulation/play`）で物理シミュレーションを
開始してください。

## パラメータ

| パラメータ | デフォルト | 意味 |
|----|----|----|
| `usd_path` | 地面のみのステージ | 環境として読み込むUSDファイル |
| `fps` | 60.0 | シミュレーターの描画・物理の更新レート [Hz] |
| `time_steps_per_second` | 600.0 | 1秒あたりの物理ソルバーステップ数。大きいほど接触や関節の挙動が安定しますが負荷が上がります |
| `real_fps` | `fps`と同じ | 実時間でのシミュレーション更新頻度 [Hz]。`fps`より小さくすると実時間よりゆっくり進みます |
| `api_port` | 8080 | REST APIサーバーのポート |
| `isaac_path` | `/isaac-sim` | Isaac Simのインストールディレクトリ |

## ランチャーのバリエーション

| 実行ファイル | 動作 |
|----|----|
| `launcher` | 標準（重力9.81 m/s²、GUIあり） |
| `launcher_with_headless` | `launcher`と同じですがGUIウィンドウなし |
| `launcher_zero_g` | 無重力（宇宙・水中ロボット向け） |
| `launcher_with_reset` | 共有メモリのトリガーでシミュレーションのタイムラインを停止・再開できます（Isaac Simを再起動せずにワールドをリセットする用途） |

## launchファイルからの使用

ROS 2のlaunchファイルから任意のUSDファイルを環境として読み込めます。

```python
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    stage_description_path = os.path.join(
        get_package_share_directory('stage_description'))

    stage_usd_path = os.path.join(stage_description_path,
                              'meshes', 'USD',
                              'stage.usd')

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher",
        parameters=[{'usd_path': str(stage_usd_path)}],
    )

    return LaunchDescription([
        isaac_launcher,
    ])
```

上記の例では"stage_description/meshes/USD/stage.usd"を環境として
Isaac Simを起動します。

## REST API

ランチャーは以下のエンドポイントを提供します（Swagger UIは
`http://localhost:8080/docs`）。通常は直接呼び出す必要はなく、
[ロボットの生成方法](./how_to_spawn_robot.md)で説明する`spawn_robot`／
`add_usd`／`publish_tf`ノードが代わりに呼び出します。

| エンドポイント | メソッド | 意味 |
|----|----|----|
| `/health` | GET | ヘルスチェック |
| `/spawn_robot` | POST | URDFをインポートし、コントローラーとセンサをセットアップ |
| `/add_usd` | POST | USDアセットをステージへ追加 |
| `/publish_tf` | POST | 追加したアセットのリンクのTFを配信 |
| `/simulation/play` | POST | シミュレーション開始 |
| `/simulation/pause` | POST | シミュレーション一時停止 |
| `/simulation/stop` | POST | シミュレーション停止 |

> **補足:** 初回のIsaac Simの起動には非常に時間がかかります
> （シェーダーのコンパイルやアセットのダウンロードが行われます）。
> ロボットをスポーンするには、Isaac Simが完全に起動している必要があります。
