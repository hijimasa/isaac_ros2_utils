# はじめに

![demo movie](../figs/shm_movie-2023-06-13_21.52.52.gif)

[このリポジトリ](https://github.com/hijimasa/isaac_ros2_utils)は、
[Isaac Sim](https://developer.nvidia.com/isaac-sim)を
[Gazebo classic](https://classic.gazebosim.org/)のように使えるようにするツール群です。
ロボットをURDFで記述し、ROS 2のlaunchファイルからスポーンし、ros2_controlで制御する——
Isaac Sim固有のコードを一切書かずに、この流れを実現できます。

## なぜこれが必要か

Isaac Simはフォトリアルなレンダリングと GPU物理演算を備えた強力なロボットシミュレーターですが、
Gazeboとは仕組みが大きく異なります。

- シーンやロボットはURDF/SDFではなく **USD**（Universal Scene Description）で記述されます。
- シミュレーションのロジックは **OmniGraph**（ビジュアルスクリプティング）と
  Isaac Sim独自の **Python API** で構築します。しかもこのAPIは
  Isaac Sim同梱のPython環境内でしか動きません。
- Gazeboの`<gazebo>`タグのような、センサ・摩擦・ジョイントゲインをURDFに書く仕組みがありません。

このリポジトリはそのギャップを埋めます。使い慣れたROS 2のワークフロー
（URDF＋launchファイル＋ros2_control＋センサトピック）はそのままに、
Isaac Simの世界への変換をパッケージ側が引き受けます。

## 仕組み

```
 ROS 2側                               Isaac Sim側
+---------------------------+         +----------------------------------+
| ros2_control              |         | Isaac Sim（"launcher"ノードが     |
|  +----------------------+ |  ROS 2  |  起動）                          |
|  | topic_based_         | | トピック |  +----------------------------+  |
|  | ros2_control         |<--------->|  | isaac_ros2_scriptsがURDFから|  |
|  | (hardware interface) | | (Joint  |  | 構築したOmniGraph           |  |
|  +----------------------+ |  State) |  | （関節・センサ・グリッパー）|  |
|                           |         |  +----------------------------+  |
| launchファイル --REST API-->| スポーン |                                  |
+---------------------------+         +----------------------------------+
```

- **isaac_ros2_scripts** はIsaac SimをROS 2ノードとして起動し、小さなREST API
  （`/spawn_robot`、`/add_usd`など）を提供します。ロボットをスポーンすると、
  URDFをUSDへ変換したうえで、関節状態やセンサデータをROS 2トピックへ配信し、
  関節指令を購読するOmniGraphのネットワークを自動的に構築します。
- **topic_based_ros2_control** はros2_controlのハードウェアインターフェースで、
  これらのトピックを介してIsaac Simと`sensor_msgs/JointState`をやり取りします。
  そのため`diff_drive_controller`や`joint_trajectory_controller`などの
  ros2_controllerがそのまま使えます。

URDFでは表現できないIsaac Sim固有の設定（ジョイントゲイン・摩擦マテリアル・
センサ・グリッパー・スラスター）は、**独自のURDFタグ**
（`<isaac_drive_api>`、`<isaac_rigid_body>`、`<isaac>`）として記述します。
各タグの書き方と、それがIsaac Sim上でどう扱われるかはチュートリアルで詳しく解説します。

## 特徴

- 任意のタイミング・位置にURDFモデルをスポーンできます。
- ros2_controlのハードウェアインターフェースを提供し、diff_drive_controllerなどの
  標準的なros2_controllerがそのまま使えます。
- 回転関節と直動関節の位置制御・速度制御に対応しています。
- 関節の状態（位置・速度・トルク）をIsaac Simからros2_controlに送信します。
- URDFの記述からセンサ（LiDAR・カメラ・デプスカメラ・接触センサ）を生成します。
- URDFの記述から吸着式グリッパー（surface gripper）とスラスターを生成します。
- URDFの記述から関節のスティフネス・ダンピング・摩擦を設定します。

## 対応バージョン

| ブランチ | Isaac Sim | ROS 2 |
|----|----|----|
| `main` | 6.x | Jazzy |
| `for-isaac-sim-5.1.0` | 5.1 | Jazzy |

## 動作環境

| 項目 | 最小スペック |
|----|--------------------|
| OS | Ubuntu 22.04 / 24.04 |
| CPU | Intel Core i7 (第7世代) <br/> AMD Ryzen 5 |
| コア数 | 4 |
| RAM | 32GB |
| ストレージ | 50GB SSD |
| GPU | GeForce RTX 3070 |
| VRAM | 8GB |
| NVIDIAドライバー | 575以降 |
| ROS 2 | Jazzy |

[Isaac Simの動作環境](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)もあわせて参照してください。

> **重要:** Isaac Sim 6はCUDA 12.9でビルドされています。ホストのドライバーが
> 古い（例: 555）場合、一見ほとんどの機能が動きますが、RTX LiDARだけが
> データを配信しなくなります（ログに `CUDA Driver CALL FAILED ... the
> provided PTX was compiled with an unsupported toolchain.` が出ます）。
> ホストのNVIDIAドライバーを575以降に更新してください。

## デモ

[サンプルリポジトリ](https://github.com/hijimasa/isaac-ros2-control-sample)で、
移動ロボットとアームロボットのデモを提供しています。Isaac SimとROS 2が共存する
Docker環境も含まれています。

以下のドキュメントを確認してください。

[移動ロボット向け](./デモ/demo_for_mobile_robot.md)

[アームロボット向け](./デモ/demo_for_arm_robot.md)

## 使い方

[チュートリアル](./チュートリアル/tutorial.md)を確認してください。

## ライセンス

このリポジトリはMITライセンスで提供されます。
[こちら](./LICENSE.md)を参照してください。
