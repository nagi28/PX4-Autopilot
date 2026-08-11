# 07. ルート A: Offboard 制御

[← 前へ: 06. 自律飛行スタック](06_autonomy_navigator.md) | [目次](README.md) | [次へ: 08. ルート B: PX4 内部に実装 →](08_route_internal.md)

---

**Offboard モード**は「外部プログラムが送る設定値で飛ぶ」フライトモードです。
コンパニオンコンピュータ（Raspberry Pi、Jetson、NUC など）や地上局から制御します。

**最も手軽に始められるルート**であり、PX4 のソースを一切変更しなくて済みます。

---

## 7.1 全体像

```
┌──────────────────────────────────────────┐
│  コンパニオンコンピュータ                     │
│   ・ROS 2 ノード                           │
│   ・MAVSDK / pymavlink                     │
│   ・独自プログラム                           │
└──────────────────┬───────────────────────┘
                   │
       ┌───────────┴────────────┐
       │                        │
  uXRCE-DDS               MAVLink
  (ROS 2, 推奨)          (MAVSDK 等)
       │                        │
       ▼                        ▼
┌──────────────────┐   ┌──────────────────┐
│ uxrce_dds_client │   │ mavlink          │
│  トピックを直接橋渡し │   │  メッセージ→トピック │
└────────┬─────────┘   └────────┬─────────┘
         │                       │
         └───────────┬───────────┘
                     ▼
      offboard_control_mode  +  trajectory_setpoint など
                     │
        ┌────────────┴────────────┐
        ▼                         ▼
   commander                mc_pos_control 以下（04 章）
   （制御フラグを立てる）        （実際の制御）
```

---

## 7.2 通信経路の選択: uXRCE-DDS vs MAVLink

| | uXRCE-DDS (ROS 2) | MAVLink |
| --- | --- | --- |
| 実装モジュール | `src/modules/uxrce_dds_client/` | `src/modules/mavlink/` |
| 扱えるトピック | **uORB トピックをほぼそのまま** | MAVLink メッセージに変換されたもののみ |
| レイテンシ | 低い | やや高い |
| 帯域効率 | 高い | 中 |
| 必要なもの | ROS 2 + `micro-xrce-dds-agent` + `px4_msgs` | MAVSDK / pymavlink など |
| 型安全 | 強い（`px4_msgs`） | 弱い |
| 推奨度 | **★ 新規開発はこちら** | 既存資産がある場合 |

> [!TIP]
> **新しく作るなら uXRCE-DDS (ROS 2) を選んでください。**
> `trajectory_setpoint` 以外の下位設定値（`vehicle_rates_setpoint`,
> `vehicle_thrust_setpoint` など）は MAVLink では送れませんが、DDS なら送れます。

---

## 7.3 uXRCE-DDS でやりとりできるトピック

定義は `src/modules/uxrce_dds_client/dds_topics.yaml` にあります。
**このファイルが ROS 2 ↔ PX4 の API 一覧そのものです。**

### PX4 へ送れるトピック（`/fmu/in/...`）

| ROS 2 トピック | 型 | 用途 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | **どの階層を制御するかの宣言（必須）** |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | **位置/速度/加速度設定値** |
| `/fmu/in/goto_setpoint` | `GotoSetpoint` | 目的地（平滑化つき） |
| `/fmu/in/vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint` | 姿勢設定値 |
| `/fmu/in/vehicle_rates_setpoint` | `VehicleRatesSetpoint` | 角速度設定値 |
| `/fmu/in/vehicle_thrust_setpoint` | `VehicleThrustSetpoint` | 推力設定値 |
| `/fmu/in/vehicle_torque_setpoint` | `VehicleTorqueSetpoint` | トルク設定値 |
| `/fmu/in/actuator_motors` | `ActuatorMotors` | **モータ直接指令** |
| `/fmu/in/actuator_servos` | `ActuatorServos` | サーボ直接指令 |
| `/fmu/in/vehicle_command` | `VehicleCommand` | **アーム、モード変更、離着陸などの指令** |
| `/fmu/in/vehicle_visual_odometry` | `VehicleOdometry` | VIO / SLAM の推定値 |
| `/fmu/in/vehicle_mocap_odometry` | `VehicleOdometry` | モーキャプの推定値 |
| `/fmu/in/obstacle_distance` | `ObstacleDistance` | 障害物距離（衝突回避用） |
| `/fmu/in/sensor_optical_flow` | `SensorOpticalFlow` | オプティカルフロー |
| `/fmu/in/distance_sensor` | `DistanceSensor` | 距離センサ |
| `/fmu/in/manual_control_input` | `ManualControlSetpoint` | 仮想 RC 入力 |
| `/fmu/in/telemetry_status` | `TelemetryStatus` | リンク状態 |

外部モード（[09 章](09_route_external_mode.md)）用:
`register_ext_component_request`, `unregister_ext_component`,
`arming_check_reply`, `setpoint_config`, `config_overrides_request`, `mode_completed`

### PX4 から受け取れるトピック（`/fmu/out/...`）

| ROS 2 トピック | 用途 |
| --- | --- |
| `/fmu/out/vehicle_local_position` | **ローカル NED 位置・速度** |
| `/fmu/out/vehicle_attitude` | **姿勢クォータニオン** |
| `/fmu/out/vehicle_global_position` | 緯度経度高度 |
| `/fmu/out/vehicle_odometry` | オドメトリ |
| `/fmu/out/vehicle_status` | **アーミング状態・フライトモード** |
| `/fmu/out/failsafe_flags` | **フェイルセーフ状態** |
| `/fmu/out/battery_status` | バッテリ |
| `/fmu/out/vehicle_land_detected` | 着地判定 |
| `/fmu/out/timesync_status` | 時刻同期 |
| `/fmu/out/sensor_combined` | 生 IMU |
| `/fmu/out/position_setpoint_triplet` | 現在のウェイポイント |
| `/fmu/out/manual_control_setpoint` | RC 入力 |

各トピックには `rate_limit` が設定されているので、必要以上に帯域を食いません。

> [!NOTE]
> **`dds_topics.yaml` を編集すれば、他の uORB トピックも公開できます。**
> ただしその場合は PX4 の再ビルドが必要で、`px4_msgs` 側にも型が必要です。

---

## 7.4 `OffboardControlMode` — 制御階層の宣言

**Offboard の要です。** 「どの階層の設定値を送るか」を宣言します。

`msg/OffboardControlMode.msg`:

```
uint64 timestamp

bool position           # trajectory_setpoint の position を使う
bool velocity           # trajectory_setpoint の velocity を使う
bool acceleration       # trajectory_setpoint の acceleration を使う
bool attitude           # vehicle_attitude_setpoint を使う
bool body_rate          # vehicle_rates_setpoint を使う
bool thrust_and_torque  # vehicle_thrust/torque_setpoint を使う
bool direct_actuator    # actuator_motors / actuator_servos を使う
```

### フラグには優先順位がある ★

`src/modules/commander/ModeUtil/control_mode.cpp` の実装:

```cpp
case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
    vehicle_control_mode.flag_control_offboard_enabled = true;

    if (offboard_control_mode.position) {
        getControlMode(SetpointType::Trajectory, vehicle_control_mode);
    } else if (offboard_control_mode.velocity) {
        getControlMode(SetpointType::Trajectory, vehicle_control_mode);
        vehicle_control_mode.flag_control_position_enabled = false;
    } else if (offboard_control_mode.acceleration) {
        /* ... */
    } else if (offboard_control_mode.attitude) {
        getControlMode(SetpointType::Attitude, vehicle_control_mode);
    } else if (offboard_control_mode.body_rate) {
        getControlMode(SetpointType::Rates, vehicle_control_mode);
    } else if (offboard_control_mode.thrust_and_torque) {
        getControlMode(SetpointType::ThrustAndTorque, vehicle_control_mode);
    } else if (offboard_control_mode.direct_actuator) {
        getControlMode(SetpointType::DirectActuators, vehicle_control_mode);
    }
    break;
```

**`else if` の連鎖なので、上の階層のフラグが立っていると下は無視されます。**

優先順位: `position` > `velocity` > `acceleration` > `attitude` > `body_rate` >
`thrust_and_torque` > `direct_actuator`

> [!WARNING]
> **よくある間違い:** 「姿勢制御したいのに `position=true` も立てたまま」
> → 位置制御として扱われ、`vehicle_attitude_setpoint` は無視されます。
> **1 つの階層だけ true にしてください。**

### 送信レート

```cpp
// src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp
bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
                      + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);
```

**`COM_OF_LOSS_T` 秒以内に `offboard_control_mode` が届いていなければ、
Offboard 信号喪失と判定されます。**

- 既定値は 1 秒程度。**安全のため 10 Hz 以上で送る**のが一般的（最低 2 Hz）
- 設定値本体（`trajectory_setpoint` など）も同じレートで送る

さらに、要求する制御に必要な推定値が有効でなければ Offboard に入れません:

| 宣言 | 必要な推定値 |
| --- | --- |
| `position` | ローカル位置が有効（`local_position_invalid` が false） |
| `velocity` | ローカル速度が有効 |
| `acceleration` / `attitude` | 姿勢が有効 |

---

## 7.5 Offboard 制御の手順（ROS 2）

### 前提

1. PX4 で `uxrce_dds_client` が動いている（SITL では自動起動）
2. コンパニオン側で `MicroXRCEAgent udp4 -p 8888` が動いている
3. ROS 2 ワークスペースに `px4_msgs` がある

### 手順

```
① offboard_control_mode と trajectory_setpoint を送り始める
       ↓  （最低 10 回程度、= 1 秒分は送る）
② VehicleCommand で Offboard モードへ切り替え
       VEHICLE_CMD_DO_SET_MODE, param1=1, param2=6
       ↓
③ VehicleCommand でアーム
       VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1
       ↓
④ 設定値を送り続ける（止めたらフェイルセーフ）
```

> [!IMPORTANT]
> **①の順序が絶対です。** 設定値を送る *前* に Offboard へ切り替えようとすると、
> `offboardCheck` に弾かれてモード変更が拒否されます。

### C++ のコード骨子

```cpp
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

// --- publisher の作成 ---
// QoS は Best Effort / KeepLast(1) が PX4 側と整合する
rclcpp::QoS qos(rclcpp::KeepLast(1));
qos.best_effort().transient_local();

auto ocm_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
                   "/fmu/in/offboard_control_mode", qos);
auto sp_pub  = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
                   "/fmu/in/trajectory_setpoint", qos);
auto cmd_pub = create_publisher<px4_msgs::msg::VehicleCommand>(
                   "/fmu/in/vehicle_command", qos);

// --- 100 ms 周期（10 Hz）で送る ---
void timer_callback()
{
    // ① 制御階層の宣言（位置制御を使う）
    px4_msgs::msg::OffboardControlMode ocm{};
    ocm.position     = true;
    ocm.velocity     = false;
    ocm.acceleration = false;
    ocm.attitude     = false;
    ocm.body_rate    = false;
    ocm.timestamp    = now_us();
    ocm_pub->publish(ocm);

    // ② 設定値（NED 座標系）
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.position = {0.0f, 0.0f, -5.0f};   // 高度 5 m（Down が負 = 上）
    sp.velocity = {NAN, NAN, NAN};       // ★ 制御しないので NaN
    sp.acceleration = {NAN, NAN, NAN};
    sp.yaw = -3.14f;                     // [rad]
    sp.yawspeed = NAN;
    sp.timestamp = now_us();
    sp_pub->publish(sp);

    // ③ 10 回送ったらモード変更＆アーム
    if (counter_ == 10) {
        publish_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);  // Offboard
        publish_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    }
    counter_++;
}

void publish_command(uint16_t command, float p1 = 0.f, float p2 = 0.f)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1  = p1;
    msg.param2  = p2;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;         // ★ 外部からの指令であることを示す
    msg.timestamp        = now_us();
    cmd_pub->publish(msg);
}
```

> [!TIP]
> **`timestamp` には PX4 の時刻（起動からのマイクロ秒）を入れてください。**
> uXRCE-DDS が時刻同期を行っており、`/fmu/out/timesync_status` からオフセットが得られます。
> `px4_ros2` ライブラリを使えば自動でやってくれます。

公式のサンプル実装は
[px4_ros_com](https://github.com/PX4/px4_ros_com) の `offboard_control.cpp` にあります。
手順の詳細は `docs/en/ros2/offboard_control.md` も参照してください。

---

## 7.6 MAVLink での Offboard

MAVLink の場合、`src/modules/mavlink/mavlink_receiver.cpp` が
メッセージを uORB トピックに変換します。

| MAVLink メッセージ | 変換先 |
| --- | --- |
| `SET_POSITION_TARGET_LOCAL_NED` | `trajectory_setpoint` + `offboard_control_mode` |
| `SET_POSITION_TARGET_GLOBAL_INT` | `trajectory_setpoint` + `offboard_control_mode` |
| `SET_ATTITUDE_TARGET` | `vehicle_attitude_setpoint` or `vehicle_rates_setpoint` |
| `SET_ACTUATOR_CONTROL_TARGET` | アクチュエータ直接 |
| `ODOMETRY` / `VISION_POSITION_ESTIMATE` | `vehicle_visual_odometry` |
| `ATT_POS_MOCAP` | `vehicle_mocap_odometry` |
| `COMMAND_LONG` / `COMMAND_INT` | `vehicle_command` |

### `type_mask` が NaN の役割を果たす

MAVLink には NaN の代わりに **ビットマスク**があります:

```cpp
setpoint.position[0] = (type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) ? (float)NAN : target_local_ned.x;
setpoint.position[1] = (type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) ? (float)NAN : target_local_ned.y;
/* ... */
```

**`_IGNORE` ビットが立っているフィールドが NaN に変換されます。**
つまり MAVLink の `type_mask` と PX4 の NaN イディオムは同じ意味です。

`offboard_control_mode` は設定値の中身から自動生成されます:

```cpp
offboard_control_mode_s
MavlinkReceiver::fill_offboard_control_mode(const trajectory_setpoint_s &setpoint)
{
    offboard_control_mode_s ocm{};
    ocm.position     = !matrix::Vector3f(setpoint.position).isAllNan();
    ocm.velocity     = !matrix::Vector3f(setpoint.velocity).isAllNan();
    ocm.acceleration = !matrix::Vector3f(setpoint.acceleration).isAllNan();
    ocm.attitude     = PX4_ISFINITE(setpoint.yaw);
    ocm.body_rate    = PX4_ISFINITE(setpoint.yawspeed);
    return ocm;
}
```

### 制限事項

| 項目 | 状況 |
| --- | --- |
| `FORCE` フラグ | **未対応**（エラーになる） |
| 座標系 | `MAV_FRAME_LOCAL_NED` / `LOCAL_OFFSET_NED` / `BODY_NED` などに限られる |
| 下位階層 | 推力・トルク・モータ直接指令は MAVLink 経由では限定的 |

MAVSDK を使う場合は `Offboard` プラグインが上記を隠蔽してくれます。

---

## 7.7 座標系の落とし穴 ★

**ROS 2 ユーザが最も高い確率で踏む地雷です。**

| | PX4 | ROS（REP 103） |
| --- | --- | --- |
| ワールド座標 | **NED**（北 - 東 - 下） | **ENU**（東 - 北 - 上） |
| 機体座標 | **FRD**（前 - 右 - 下） | **FLU**（前 - 左 - 上） |
| 高度 | z が下向き正（上昇 = z が負） | z が上向き正 |
| 方位 | 北から時計回り | 東から反時計回り |

**PX4 のトピックに直接 publish する場合、自動変換は一切されません。**

```
高度 5 m でホバリング  →  position = {0, 0, -5.0}     ← z は負
北へ 2 m/s            →  velocity = {2.0, 0, 0}
東へ 2 m/s            →  velocity = {0, 2.0, 0}
上昇 1 m/s            →  velocity = {0, 0, -1.0}     ← 負
```

変換のヘルパは
[px4_ros_com](https://github.com/PX4/px4_ros_com) の `frame_transforms.cpp` にあります。
詳細は [付録 A2](A2_frames_params.md) を参照してください。

---

## 7.8 Offboard の限界と注意点

### 制御権を失うことがある

フェイルセーフが発動すると `nav_state` が変わり、
`flag_control_offboard_enabled` が false になります。
**外部プログラムは気づかずに設定値を送り続けてしまいます。**

対策:

```cpp
// vehicle_status を購読して監視する
if (vehicle_status.nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
    // 制御権を失っている。再取得を試みるか、安全に停止する
}

// failsafe_flags で原因を知る
```

### アーミングチェックに関与できない

Offboard モードは PX4 にとって「よく分からない外部が制御している状態」です。
**外部プログラムの内部状態（経路計画が失敗した、センサが死んだ等）を
PX4 のアーミングチェックに反映させる手段がありません。**

これを解決するのが [09 章の外部モード](09_route_external_mode.md)です。

### モードとして表示されない

GCS には単に "Offboard" としか出ません。
「探索モード」「配送モード」など複数の自律機能を作っても、
運用者からは区別できません。これも外部モードなら解決します。

### まとめ: Offboard が向いている場面 / 向かない場面

| 向いている | 向かない |
| --- | --- |
| プロトタイピング、実験 | プロダクション運用 |
| 単一の自律機能 | 複数モードの切り替え |
| 研究用途 | 認証が必要なシステム |
| 既に MAVLink 資産がある | 安全機構との統合が必須 |

---

## 7.9 SITL での試し方

```sh
# ターミナル 1: PX4 SITL
make px4_sitl gz_x500

# ターミナル 2: XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# ターミナル 3: 自分の ROS 2 ノード
ros2 run my_package my_offboard_node

# トピックが見えているか確認
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position
```

PX4 の pxh コンソール側での確認:

```sh
pxh> listener offboard_control_mode
pxh> listener trajectory_setpoint
pxh> listener vehicle_control_mode
pxh> commander status
pxh> uxrce_dds_client status
```

詳細な SITL 手順は [11 章](11_sitl_workflow.md) にあります。

---

## 7.10 この章のまとめ

- Offboard は **PX4 を改造せずに外部から制御する**最も手軽なルート
- 通信は **uXRCE-DDS (ROS 2) 推奨**、MAVLink も可
- `dds_topics.yaml` が ROS 2 ↔ PX4 の API 一覧
- **`OffboardControlMode` で制御階層を宣言する。フラグには優先順位があり、1 つだけ立てる**
- `COM_OF_LOSS_T` 秒以内に送り続けないと信号喪失と判定される
- **設定値を送り始めてからモード変更 → アーム、の順序が絶対**
- **NED / FRD 座標系。ROS の ENU / FLU から必ず変換する**
- 制御権を失う可能性があるので `vehicle_status` を監視する
- 本格運用には [外部モード](09_route_external_mode.md) を検討する

---

[← 前へ: 06. 自律飛行スタック](06_autonomy_navigator.md) | [目次](README.md) | [次へ: 08. ルート B: PX4 内部に実装 →](08_route_internal.md)
