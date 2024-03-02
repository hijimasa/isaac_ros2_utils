# はじめに

![demo movie](../figs/shm_movie-2023-06-13_21.52.52.gif)

[このリポジトリ](https://github.com/hijimasa/isaac_ros2_utils)は、[Gazebo Classic](ttps://classic.gazebosim.org/)のような使いやすいユーティリティツールを[Isaac Sim](https://developer.nvidia.com/isaac-sim)で提供するためのものです。

## 特徴

- URDFモデルを任意のタイミングと位置姿勢で生成できます。
- ros2_controller（例：diff_drive_controller）を使用するためのros2_controlプラグインを提供します。
- 位置制御と速度制御を使った回転関節と角柱関節をサポートしています。
- Isaac Simからros2_controlに関節の状態(位置、速度、力)を送信します。
- URDFの記述からセンサを起動します。
- センサとコントローラを任意のタイミングで起動できます。

## システム要件

| 要素 | 最小スペック |
|----|--------------------|
| OS | Ubuntu 22.04 |
| CPU | Intel Core i7 (7th Generation) <br/> AMD Ryzen 5 |
| Cores | 4 |
| RAM | 32GB |
| Storage | 50GB SSD |
| GPU | GeForce RTX 2070 |
| VRAM | 8GB |
| ROS 2 | Humble |

[こちら](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements)のIsaac Simのシステム要件を参照してください。

## デモ

[サンプルリポジトリ](https://github.com/hijimasa/isaac-ros2-control-sample)で移動ロボットとアームロボットのデモンストレーションを提供しています。

使い方は下記のドキュメントを参照ください。

- [移動ロボット向け](./デモ/demo_for_mobile_robot.md)

- [アームロボット向け](./デモ/demo_for_arm_robot.md)

## 使い方

[こちら](./チュートリアル/tutorial.md)のドキュメントを参照ください。
        
## ライセンス

このリポジトリはMITライセンスで提供されます。
詳細は[こちら](./LICENCE.md)を参照ください。
