# 11. SITL と開発ワークフロー

[← 前へ: 10. 3 ルートの比較と選定](10_route_comparison.md) | [目次](README.md) | [次へ: 付録 A1. 主要 uORB トピック早見表 →](A1_uorb_topics.md)

---

**自律制御の開発は 9 割が SITL（Software In The Loop）で進みます。**
実機で試すのは最後の確認だけです。この章では開発サイクルの回し方をまとめます。

---

## 11.1 SITL とは

SITL は **PX4 のフライトスタックを PC 上でそのまま動かす**仕組みです。

```
┌──────────────────────────────────────────┐
│  PX4 バイナリ（実機と同じフライトスタック）    │
│                                           │
│  ekf2 / mc_pos_control / commander / …     │
│           ↑ センサ値      ↓ モータ指令       │
│  ┌─────────────────────────────────────┐  │
│  │ simulator モジュール                  │  │
│  │  （ドライバの代わり）                   │  │
│  └─────────────────────────────────────┘  │
└──────────────────┬───────────────────────┘
                   │ TCP/UDP
                   ▼
        ┌─────────────────────────┐
        │  シミュレータ               │
        │  Gazebo / jMAVSim / SIH   │
        │  物理演算 + センサモデル     │
        └─────────────────────────┘
```

**重要**: 制御・推定・安全機構のコードは実機と完全に同じです。
SITL で正しく飛べば、制御ロジックは実機でも同じように動きます
（センサノイズやモデル誤差の違いは残ります）。

---

## 11.2 シミュレータの選択

| シミュレータ | 起動 | 特徴 |
| --- | --- | --- |
| **Gazebo (gz)** | `make px4_sitl gz_x500` | **標準・推奨**。センサ・カメラ・環境が充実 |
| jMAVSim | `make px4_sitl jmavsim_iris` | 軽量。マルチコプターのみ |
| SIH | `make px4_sitl sihsim_quadx` | **外部シミュレータ不要**。PX4 内部で物理演算 |
| Gazebo Classic | `make px4_sitl gazebo-classic_iris` | 旧世代。既存資産がある場合 |
| なし | `make px4_sitl none_iris` | シミュレータなしで PX4 だけ起動（コンソール確認用） |

エアフレーム定義は `ROMFS/px4fmu_common/init.d-posix/airframes/` にあります。

```sh
ls ROMFS/px4fmu_common/init.d-posix/airframes/ | grep gz_
# 4001_gz_x500          基本のクアッドコプター
# 4002_gz_x500_depth    深度カメラ付き
# 4005_gz_x500_vision   Vision（外部位置推定）用
# 4010_gz_x500_mono_cam カメラ付き
# 4013_gz_x500_lidar_2d 2D LiDAR 付き
# 4016_gz_x500_lidar_down 下向き LiDAR
# 4019_gz_x500_gimbal   ジンバル付き
```

> [!TIP]
> **自律制御の開発なら `gz_x500` から始めてください。**
> SLAM や Vision を使うなら `gz_x500_vision` や `gz_x500_depth`、
> 障害物回避なら `gz_x500_lidar_2d` が近道です。

Gazebo のモデルは git submodule（`Tools/simulation/gz`、
[PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models)）にあります。
初回は submodule の取得が必要です:

```sh
git submodule update --init --recursive
```

---

## 11.3 基本の開発サイクル

```sh
# 1. ビルド + 起動（差分ビルドなら 10〜30 秒）
make px4_sitl gz_x500

# 2. pxh> プロンプトが出たら操作できる
pxh> commander takeoff
pxh> commander land

# 3. コードを直したら Ctrl+C → 再度 make
```

### よく使うビルドコマンド

```sh
make px4_sitl_default          # ビルドのみ（シミュレータを起動しない）
make px4_sitl gz_x500          # ビルド + Gazebo 起動
make px4_sitl none_iris        # ビルド + PX4 のみ起動（コンソール確認用）

make list_config_targets       # 選べるターゲット一覧
make px4_sitl_default boardconfig   # モジュールの ON/OFF を対話的に編集

make clean                     # ビルド成果物を削除
make distclean                 # submodule も含めて完全リセット

make format                    # コード整形（コミット前に必須）
make check_format              # CI と同じチェック
make tests                     # 単体テスト
```

ヘッドレス実行（GUI なしのサーバ等）:

```sh
HEADLESS=1 make px4_sitl gz_x500
```

---

## 11.4 pxh コンソールでの操作

SITL を起動すると `pxh>` プロンプトが出ます。実機の NuttX シェルと同じコマンドが使えます。

### 飛行の操作

```sh
commander takeoff              # 離陸
commander land                 # 着陸
commander arm                  # アーム
commander disarm               # 武装解除
commander mode auto:mission    # ミッションモードへ
commander mode offboard        # Offboard モードへ
commander mode posctl          # Position モードへ
commander status               # 状態表示
```

### 状態の確認（[02 章](02_uorb_runtime.md#27-実機と-sitl-で使う調査コマンド)の再掲）

```sh
listener vehicle_local_position     # 位置・速度
listener vehicle_attitude           # 姿勢
listener trajectory_setpoint        # 設定値
listener vehicle_control_mode       # どの制御段が動いているか
listener failsafe_flags             # フェイルセーフ状態

uorb top                            # トピックのレート
uorb top -a                         # 全トピック
work_queue status                   # WorkQueue の負荷
top                                 # CPU / メモリ
perf                                # 性能カウンタ
dmesg                               # 起動ログ
```

### パラメータ操作

```sh
param show MPC_XY_P                 # 表示
param show MPC_*                    # ワイルドカード
param set MPC_XY_P 1.2              # 変更（即座に反映される）
param save                          # 永続化
param reset MPC_XY_P                # 既定値に戻す
param reset_all                     # 全リセット
param status                        # 使用状況
```

> [!TIP]
> **`param set` はモジュールに即座に反映されます**（`parameter_update` トピック経由）。
> ゲイン調整をリアルタイムで試せるので、SITL でのチューニングが速く回ります。

SITL のパラメータは `build/px4_sitl_default/rootfs/` に保存され、
再起動しても残ります。リセットしたいときはこのディレクトリを消してください。

### モジュールの起動・停止

```sh
mc_pos_control stop
my_controller start
my_controller status
```

**実行時にモジュールを差し替えられる**のが uORB アーキテクチャの利点です。

---

## 11.5 QGroundControl との接続

SITL は自動的に MAVLink を UDP で公開します。
QGroundControl を同じ PC で起動すれば自動接続されます。

```
PX4 SITL ─── UDP 14550 ───> QGroundControl（GCS 用）
          └── UDP 14540 ───> MAVSDK / pymavlink（開発者用）
```

> [!IMPORTANT]
> **既定では GCS か RC のどちらかが繋がっていないとアームできません。**
> これは「常に手動に戻せる」ための安全機構です。
> SITL で QGC を使わない場合は、`COM_RC_IN_MODE` などの調整が必要です
> （**実機では絶対に外さないこと**）。

---

## 11.6 ROS 2 との接続

```sh
# ターミナル 1: PX4 SITL
make px4_sitl gz_x500

# ターミナル 2: XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# ターミナル 3: ROS 2 側
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position
```

PX4 側の確認:

```sh
pxh> uxrce_dds_client status
```

`uxrce_dds_client` は SITL では自動起動します
（`ROMFS/px4fmu_common/init.d-posix/rcS` を参照）。

必要なリポジトリ:

| リポジトリ | 内容 |
| --- | --- |
| [px4_msgs](https://github.com/PX4/px4_msgs) | ROS 2 メッセージ型（**必須**） |
| [px4_ros_com](https://github.com/PX4/px4_ros_com) | サンプルと座標変換ヘルパ |
| [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) | 外部モード用ライブラリ |

> [!WARNING]
> **`px4_msgs` のバージョンは PX4 のバージョンと揃えてください。**
> `msg/versioned/` のメッセージには `MESSAGE_VERSION` があり、
> 不一致だと通信できません。PX4 の main ブランチを使うなら `px4_msgs` も main を使います。

---

## 11.7 ログ（ULog）と解析

**制御の検証はログ解析でしかできません。** 目視では分かりません。

### ログの場所

```
build/px4_sitl_default/rootfs/log/<日付>/<時刻>.ulg
```

実機では SD カードの `/fs/microsd/log/` に保存されます。

### ログ設定

| パラメータ | 意味 |
| --- | --- |
| `SDLOG_MODE` | いつ記録するか（アーム時のみ / 起動から / …） |
| `SDLOG_PROFILE` | **記録するトピックのプロファイル**（既定 / 高レート / EKF リプレイ / …） |
| `SDLOG_MAX_SIZE` | 1 ファイルの最大サイズ |
| `SDLOG_DIRS_MAX` | 保持するログ数 |

> [!TIP]
> **制御の詳細を見たいなら `SDLOG_PROFILE` に高レート系のビットを立ててください。**
> 既定では角速度レベルのデータが間引かれています。

### 解析ツール

| ツール | 用途 |
| --- | --- |
| [Flight Review](https://review.px4.io/) | **ブラウザにアップロードするだけ**。最も手軽 |
| [PlotJuggler](https://github.com/facontidavide/PlotJuggler) | 対話的なプロット。ULog を直接読める |
| [pyulog](https://github.com/PX4/pyulog) | Python での解析（`ulog2csv`, `ulog_info`） |
| `Tools/ecl_ekf/` | **EKF の解析スクリプト**（推定の健全性チェック） |

```sh
pip install pyulog
ulog_info log.ulg              # 含まれるトピック一覧
ulog2csv log.ulg               # CSV に変換
```

### 自律制御のデバッグで見るべきトピック

| 見たいこと | トピック |
| --- | --- |
| 設定値が届いているか | `trajectory_setpoint`, `offboard_control_mode` |
| 制御段が動いているか | `vehicle_control_mode` |
| 位置追従の誤差 | `vehicle_local_position` vs `vehicle_local_position_setpoint` |
| 姿勢追従の誤差 | `vehicle_attitude` vs `vehicle_attitude_setpoint` |
| 角速度追従の誤差 | `vehicle_angular_velocity` vs `vehicle_rates_setpoint` |
| 推力が飽和していないか | `actuator_motors`, `control_allocator_status` |
| 積分が溜まっていないか | `rate_ctrl_status` |
| 推定の健全性 | `estimator_status`, `estimator_innovations`, `estimator_status_flags` |
| なぜフェイルセーフしたか | `failsafe_flags`, `vehicle_status` |

> [!TIP]
> **「設定値」と「実際」を重ねてプロットするのが基本です。**
> 位置がずれているのに姿勢は設定値どおりなら、位置制御ゲインの問題。
> 姿勢もずれているなら、より内側（角速度、推力配分、機体そのもの）の問題です。
> **カスケードの外側から内側へ切り分けていきます。**

---

## 11.8 単体テスト

PX4 は Google Test を組み込んでいます。**制御則の開発では単体テストが最速です。**

```sh
make tests                              # 全テスト
make tests TESTFILTER=PositionControl   # 絞り込み
```

既存のテスト例:

| テスト | 対象 |
| --- | --- |
| `src/modules/mc_pos_control/PositionControl/PositionControlTest.cpp` | 位置制御則 |
| `src/modules/mc_pos_control/PositionControl/ControlMathTest.cpp` | 推力→姿勢変換 |
| `src/modules/mc_pos_control/Takeoff/TakeoffTest.cpp` | 離陸ステートマシン |
| `src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp` | 姿勢制御則 |
| `src/lib/control_allocation/control_allocation/*Test.cpp` | 推力配分 |
| `src/modules/commander/failsafe/failsafe_test.cpp` | **フェイルセーフ状態機械** |

> [!TIP]
> **`failsafe_test.cpp` は読む価値があります。**
> 「どの条件でどのアクションが起きるか」がテストケースとして列挙されており、
> [05 章](05_modes_safety.md)の内容の実行可能な仕様書になっています。

新しい制御則を書くときは、**まず単体テストで数値を確認**してから SITL に持っていってください。
SITL で「なんとなく飛ばない」を追いかけるより圧倒的に速いです。

---

## 11.9 コーディング規約とコミット前チェック

```sh
make format          # C/C++ の自動整形
make check_format    # CI と同じチェック（整形されていなければ失敗）
make tests           # 単体テスト
make quick_check     # SITL ビルド + 実機ビルド + テスト + 整形チェック
```

CI は `make check_format` を強制します。**コミット前に `make format` を必ず実行してください。**

---

## 11.10 よくあるトラブルと対処

| 症状 | 確認すること |
| --- | --- |
| `command not found` | `.px4board` に `CONFIG_MODULES_...=y` を追加したか |
| モジュールが動かない | `init()` で `registerCallback()` を呼んだか |
| 設定値が効かない | `listener vehicle_control_mode` でフラグを確認 |
| Offboard に入れない | 設定値を送り始めてからモード変更しているか。`COM_OF_LOSS_T` |
| アームできない | `commander status` と `listener health_report` で理由を確認 |
| 位置制御に入れない | `listener estimator_status_flags`、GPS / 外部Vision の状態 |
| 勝手に RTL / Land する | `listener failsafe_flags` で原因特定 |
| 空中で武装解除される | `land_detector` の誤判定。`COM_DISARM_LAND` |
| 機体が振動する | ジャイロフィルタ（`IMU_GYRO_CUTOFF`）、D ゲイン |
| ROS 2 でトピックが見えない | XRCE-DDS Agent が起動しているか、`px4_msgs` のバージョン |
| ビルドが通らない | `git submodule update --init --recursive`、`make distclean` |
| SITL のパラメータが変 | `build/px4_sitl_default/rootfs/` を削除して再起動 |

---

## 11.11 実機へ移行するときのチェックリスト

SITL で動いてから実機へ行く前に:

- [ ] エアフレーム設定（`SYS_AUTOSTART`）が正しい
- [ ] `CA_*` パラメータでモータ配置が正しく定義されている
- [ ] センサキャリブレーション（加速度計、ジャイロ、磁気、RC）が完了している
- [ ] **RC 送信機でモード切替とキルスイッチが機能する**
- [ ] ジオフェンス（`GF_ACTION`, `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`）を設定した
- [ ] フェイルセーフ（`NAV_RCL_ACT`, `NAV_DLL_ACT`, `COM_OBL_RC_ACT`）を確認した
- [ ] SITL 用に緩めたパラメータを**すべて元に戻した**
- [ ] ログ記録が有効（`SDLOG_MODE`）
- [ ] 手動モード（Stabilized / Position）で問題なく飛ぶことを先に確認した
- [ ] 屋外なら十分に広い場所、屋内ならテザーまたはネット

> [!WARNING]
> **自律制御の初回テストでは、必ず人間が RC を握ってください。**
> モードスイッチ 1 つで手動に戻れる状態を保つこと。
> 高度は低く、ジオフェンスは狭く。徐々に広げていきます。

---

## 11.12 この章のまとめ

- **開発の 9 割は SITL**。制御・推定・安全機構のコードは実機と同一
- `make px4_sitl gz_x500` が標準。用途に応じて `_vision` `_depth` `_lidar_2d` を選ぶ
- `pxh>` コンソールの `listener` / `uorb top` / `param` / `commander` が主力
- **`param set` は即座に反映される**ので、SITL でのチューニングが速い
- ROS 2 連携には XRCE-DDS Agent と `px4_msgs`（**バージョンを揃える**）
- **検証はログ（ULog）で行う**。設定値と実際を重ねてプロットする
- カスケードの**外側から内側へ**切り分ける
- 制御則の開発は**単体テストが最速**
- コミット前に `make format`
- **実機移行前にチェックリストを通す。手動復帰手段を必ず確保する**

---

[← 前へ: 10. 3 ルートの比較と選定](10_route_comparison.md) | [目次](README.md) | [次へ: 付録 A1. 主要 uORB トピック早見表 →](A1_uorb_topics.md)
