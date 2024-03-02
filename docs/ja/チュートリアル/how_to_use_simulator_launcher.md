# シミュレーションランチャーの使い方

Gazebo Classicでも、ROS 2のlaunchファイルからシミュレータを起動できたように、このリポジトリではシミュレーションランチャーを使用してIsaac Simを起動できます。

## コマンドラインからの実行

シミュレーションランチャーは以下のコマンドで実行できます。

```bash
ros2 run isaac_ros2_scripts launcher
```

このコマンドでは、地面のあるデフォルトの環境が読み込まれるようになっています。

## launchファイルからの実行

ROS 2のlaunchファイルを使用することで、任意のUSDファイルを読み込んで起動させることが出来ます。
launchファイルの例を以下に示します。

```python
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    core_stage_description_path = os.path.join(
        get_package_share_directory('stage_description'))

    core_stage_usd_path = os.path.join(stage_description_path,
                              'meshes', 'USD',
                              'stage.usd')

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher",
        parameters=[{'usd_path': str(core_stage_usd_path)}],
    )
    
    return LaunchDescription([
        isaac_launcher,
    ])
```

上記の例では、stage_description/meshes/USD/stage.usdファイルを読み込んで、シミュレーションランチャーを起動しています。