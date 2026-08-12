# 付録 A1. 主要 uORB トピック早見表

[← 前へ: 11. SITL と開発ワークフロー](11_sitl_workflow.md) | [目次](README.md) | [次へ: 付録 A2. 座標系とパラメータ早見表 →](A2_frames_params.md)

---

自律制御の開発でよく触るトピックをまとめます。
**`✓ROS2`** 列は `src/modules/uxrce_dds_client/dds_topics.yaml` に登録済み
（= ROS 2 から読み書きできる）ことを示します。

メッセージ定義は `msg/` または `msg/versioned/` にあります。
**`msg/versioned/` のものは互換性が保証されており、外部連携ではこちらを使うべきです。**

---

## A1.1 状態推定（PX4 → 自分）

| トピック | 定義 | 主な publisher | 内容 | ✓ROS2 |
| --- | --- | --- | --- | --- |
| `vehicle_local_position` | `versioned/` | `ekf2` | **ローカル NED 位置・速度・加速度、妥当性フラグ、リセットカウンタ** | out |
| `vehicle_attitude` | `versioned/` | `ekf2` | **姿勢クォータニオン（NED→body）** | out |
| `vehicle_global_position` | `versioned/` | `ekf2` | 緯度経度・高度（AMSL） | out |
| `vehicle_odometry` | `versioned/` | `ekf2` | 位置・姿勢・速度をまとめたオドメトリ | out |
| `vehicle_angular_velocity` | `versioned/` | `sensors` | **制御用角速度（フィルタ済）+ 角加速度** | (コメントアウト) |
| `vehicle_acceleration` | `msg/` | `sensors` | 制御用加速度 | – |
| `vehicle_imu` | `msg/` | `sensors` | **EKF2 への入力**。瞬時値ではなく積分増分（Δθ, Δv）＋各積分期間 | – |
| `sensor_combined` | `msg/` | `sensors` | 生ジャイロ・加速度（ログ／互換用） | out |
| `vehicle_air_data` | `msg/` | `sensors` | 気圧高度・温度 | – |
| `vehicle_magnetometer` | `msg/` | `sensors` | 磁気 | – |
| `vehicle_gps_position` | `msg/` | `sensors` | GNSS | – |
| `estimator_status` | `msg/` | `ekf2` | 革新量とテスト比 | – |
| `estimator_status_flags` | `msg/` | `ekf2` | **どの補正源が有効かのビットフラグ** | out |
| `estimator_innovations` | `msg/` | `ekf2` | 各センサの革新量 | – |
| `estimator_sensor_bias` | `msg/` | `ekf2` | 推定されたセンサバイアス | – |
| `wind` | `versioned/` | `ekf2` | 推定風速 | – |

### `vehicle_local_position` の重要フィールド

```
x, y, z              位置 [m]（NED）
vx, vy, vz           速度 [m/s]（NED）    ★ 位置の微分ではなく EKF2 の独立した推定状態
ax, ay, az           加速度 [m/s²]（NED）  ※ mc_pos_control は使わない（下記）
heading              方位 [rad]

xy_valid, z_valid            位置が有効か  ★制御前に確認する
v_xy_valid, v_z_valid        速度が有効か
xy_global, z_global          緯度経度・標高への変換基準があるか
dead_reckoning               推測航法中（絶対位置補正なし）

eph, epv                     位置の標準偏差 [m]
evh, evv                     速度の標準偏差 [m/s]

xy_reset_counter, z_reset_counter, heading_reset_counter   ★リセット検知用
delta_xy, delta_z, delta_heading                           ★ジャンプ量

dist_bottom, dist_bottom_valid   地面までの距離
ref_lat, ref_lon, ref_alt        ローカル原点の緯度経度高度
```

> [!NOTE]
> **1 トピックに位置・速度・加速度が同梱されています。**
> 位置制御は位置だけでなく速度もフィードバックしており、
> 実際には速度 PID が主役です。
>
> ただし **`ax/ay/az` は EKF の推定状態ではありません。**
> 24 誤差状態に加速度は含まれず、この値は
> 「加速度計の Δv をバイアス補正 → NED 回転 → 重力除去 → 平均化」しただけの
> **フィルタを通っていない生値の加工**です
> （`OutputPredictor::getVelocityDerivative()`。[03 章](03_sensing_estimation.md#3-つの出力の出自は違う)）。
>
> そのため **`mc_pos_control` はこのフィールドを読んでいません**。
> 位置制御の D 項に使う加速度は、制御器が
> 「フィルタ済み速度を自分で微分する」形で作っています
> （[04 章](04_control_cascade.md#_vel_dot-はどこから来るのか)）。
> `ax/ay/az` の主な用途はログと他モジュールでの参照です。
>
> 一方 **`x/y/z` と `vx/vy/vz` は正真正銘のカルマン推定状態**（`pos` / `vel`）で、
> 出力予測器が現在時刻へ外挿したものです。

---

## A1.2 設定値（自分 → PX4）★ 最重要

**04 章の制御カスケードの各段に対応します。上ほど上流。**

| トピック | 定義 | 通常の publisher | 内容 | ✓ROS2 |
| --- | --- | --- | --- | --- |
| `goto_setpoint` | `versioned/` | 外部 | **目的地（PX4 側で平滑化）** | in |
| `position_setpoint_triplet` | `msg/` | `navigator` | 前・現・次のウェイポイント（緯度経度） | in/out |
| `trajectory_setpoint` | `versioned/` | `flight_mode_manager` / 外部 | **位置・速度・加速度・yaw（NED）** | in |
| `vehicle_attitude_setpoint` | `versioned/` | `mc_pos_control` / 外部 | **姿勢クォータニオン + 推力** | in |
| `vehicle_rates_setpoint` | `versioned/` | `mc_att_control` / 外部 | **角速度 + 推力** | in |
| `vehicle_thrust_setpoint` | `msg/` | `mc_rate_control` / 外部 | 正規化推力ベクトル [-1,1] | in |
| `vehicle_torque_setpoint` | `msg/` | `mc_rate_control` / 外部 | 正規化トルクベクトル [-1,1] | in |
| `actuator_motors` | `versioned/` | `control_allocator` / 外部 | **各モータへの直接指令** | in |
| `actuator_servos` | `versioned/` | `control_allocator` / 外部 | 各サーボへの直接指令 | in |
| `offboard_control_mode` | `msg/` | 外部 | **どの階層を制御するかの宣言** | in |

### `trajectory_setpoint`（`msg/versioned/TrajectorySetpoint.msg`）

```
float32[3] position       # [m] NED。制御しないなら NaN
float32[3] velocity       # [m/s] NED
float32[3] acceleration   # [m/s²] NED
float32[3] jerk           # ログ用のみ
float32 yaw               # [rad] -PI..+PI
float32 yawspeed          # [rad/s]
```

> **NaN = その自由度は制御しない。** x と y は必ずペアで指定する。
> 詳細は [04 章の NaN イディオム](04_control_cascade.md#nan-イディオム)。

### `offboard_control_mode`（`msg/OffboardControlMode.msg`）

```
bool position           # trajectory_setpoint の position を使う
bool velocity           # 〃 velocity
bool acceleration       # 〃 acceleration
bool attitude           # vehicle_attitude_setpoint を使う
bool body_rate          # vehicle_rates_setpoint を使う
bool thrust_and_torque  # vehicle_thrust/torque_setpoint を使う
bool direct_actuator    # actuator_motors / actuator_servos を使う
```

> **優先順位あり（上から順に評価される）。1 つだけ true にすること。**

---

## A1.3 モードと安全

| トピック | 定義 | publisher | 内容 | ✓ROS2 |
| --- | --- | --- | --- | --- |
| `vehicle_status` | `versioned/` | `commander` | **`nav_state`（フライトモード）、`arming_state`、機体タイプ** | out |
| `vehicle_control_mode` | `msg/` | `commander` | **各制御段の ON/OFF フラグ** | out |
| `failsafe_flags` | `msg/` | `commander` | **何の異常が起きているか** | out |
| `vehicle_command` | `versioned/` | 外部 / GCS | **アーム、モード変更、離着陸などの指令** | in |
| `vehicle_command_ack` | `versioned/` | `commander` | コマンドへの応答 | out |
| `vehicle_land_detected` | `versioned/` | `land_detector` | 着地・接地・自由落下の判定 | out |
| `home_position` | `versioned/` | `commander` | ホーム位置 | out |
| `health_report` | `msg/` | `commander` | ヘルスチェックの結果 | – |
| `failure_detector_status` | `msg/` | `commander` | 姿勢異常などの検出 | out |
| `actuator_armed` | `msg/` | `commander` | アーミング状態（低レベル） | – |
| `manual_control_setpoint` | `versioned/` | `manual_control` | RC スティック入力 | in/out |
| `takeoff_status` | `msg/` | `mc_pos_control` | 離陸シーケンスの状態 | – |
| `mission_result` | `msg/` | `navigator` | ミッションの進捗 | – |
| `geofence_result` | `msg/` | `navigator` | ジオフェンス判定 | – |
| `rtl_status` | `msg/` | `navigator` | RTL の状態 | – |

### `vehicle_command` でよく使うコマンド

`msg/versioned/VehicleCommand.msg` より:

| 定数 | 値 | 用途 | パラメータ |
| --- | --- | --- | --- |
| `VEHICLE_CMD_DO_SET_MODE` | 176 | **モード変更** | `param1=1`, `param2=`カスタムモード番号 |
| `VEHICLE_CMD_COMPONENT_ARM_DISARM` | 400 | **アーム/解除** | `param1=1`（アーム）/ `0`（解除） |
| `VEHICLE_CMD_NAV_TAKEOFF` | 22 | 離陸 | `param7=`高度 |
| `VEHICLE_CMD_NAV_LAND` | 21 | 着陸 | — |

カスタムモード番号（`src/modules/commander/px4_custom_mode.h`）:

| 番号 | モード |
| --- | --- |
| 1 | Manual |
| 2 | Altitude |
| 3 | Position |
| 4 | Auto（`param3` でサブモード指定） |
| 5 | Acro |
| **6** | **Offboard** |
| 7 | Stabilized |
| 10 | Termination |
| 11 | Altitude Cruise |

> `from_external = true` を設定してください（外部からの指令であることを示す）。

---

## A1.4 外部から PX4 へ情報を与えるトピック

| トピック | 定義 | 用途 | ✓ROS2 |
| --- | --- | --- | --- |
| `vehicle_visual_odometry` | `versioned/VehicleOdometry` | **VIO / SLAM の推定値を EKF2 へ** | in |
| `vehicle_mocap_odometry` | `versioned/VehicleOdometry` | **モーションキャプチャの推定値を EKF2 へ** | in |
| `obstacle_distance` | `msg/` | 障害物距離（衝突回避へ） | in |
| `distance_sensor` | `msg/` | 距離センサの値 | in |
| `sensor_optical_flow` | `msg/` | オプティカルフロー | in |
| `manual_control_input` | `versioned/ManualControlSetpoint` | 仮想 RC 入力 | in |
| `telemetry_status` | `msg/` | リンク品質の申告 | in |
| `onboard_computer_status` | `msg/` | コンパニオンの状態 | in |

---

## A1.5 外部モード（ルート C）用トピック

| トピック | 定義 | 方向 | 用途 |
| --- | --- | --- | --- |
| `register_ext_component_request` | `versioned/` | in | **モード登録要求** |
| `register_ext_component_reply` | `versioned/` | out | **モード ID の割り当て** |
| `unregister_ext_component` | `versioned/` | in | 登録解除 |
| `arming_check_request` | `versioned/` | out | **アーミングチェック要求（定期）** |
| `arming_check_reply` | `versioned/` | in | **チェック結果と必要条件の申告** |
| `setpoint_config` | `versioned/` | in | **送る設定値の種類を宣言** |
| `setpoint_config_reply` | `versioned/` | out | 宣言への応答 |
| `mode_completed` | `versioned/` | in/out | **モード完了の通知** |
| `config_overrides_request` | `versioned/ConfigOverrides` | in | 自動武装解除やフェイルセーフの一時上書き |
| `vehicle_command_mode_executor` | `versioned/VehicleCommand` | in | モードエグゼキュータからの指令 |

---

## A1.6 診断・状態監視

| トピック | 内容 |
| --- | --- |
| `rate_ctrl_status` | 角速度制御の積分項の値 |
| `control_allocator_status` | **配分できなかったトルク・推力（飽和の指標）** |
| `actuator_controls_status` | アクチュエータ制御の統計 |
| `hover_thrust_estimate` | 推定されたホバリング推力 |
| `battery_status` | バッテリ電圧・電流・残量 |
| `esc_status` | ESC の回転数・温度・エラー |
| `cpuload` | CPU 負荷 |
| `logger_status` | ロガーの状態 |
| `sensor_selection` | どのセンサが選ばれているか |
| `estimator_selector_status` | どの EKF2 インスタンスが選ばれているか |
| `vehicle_imu_status` | IMU の健全性・クリッピング |
| `timesync_status` | **ROS 2 との時刻同期オフセット** |

---

## A1.7 チートシート: 「これを調べたい」→「このトピック」

| 調べたいこと | コマンド |
| --- | --- |
| 今どこにいるか | `listener vehicle_local_position` |
| どちらを向いているか | `listener vehicle_attitude` |
| どのモードか、アーム状態は | `listener vehicle_status` / `commander status` |
| **なぜ制御が効かないのか** | `listener vehicle_control_mode` |
| **なぜフェイルセーフしたのか** | `listener failsafe_flags` |
| 設定値が届いているか | `listener trajectory_setpoint` |
| Offboard が生きているか | `listener offboard_control_mode` |
| 推定は健全か | `listener estimator_status_flags` / `estimator_status` |
| モータが飽和していないか | `listener control_allocator_status` |
| 着地判定はどうなっているか | `listener vehicle_land_detected` |
| トピックのレートは | `uorb top` |
| どのモジュールが重いか | `perf` / `work_queue status` / `top` |

---

## A1.8 トピック定義の探し方

```sh
# メッセージ定義を探す
ls msg/ msg/versioned/ | grep -i local_position

# 誰が publish しているか
grep -rn "uORB::Publication.*ORB_ID(trajectory_setpoint)" src/

# 誰が subscribe しているか
grep -rn "uORB::Subscription.*ORB_ID(trajectory_setpoint)" src/

# ROS 2 に公開されているか
grep -n "trajectory_setpoint" src/modules/uxrce_dds_client/dds_topics.yaml
```

**この 4 つの grep で、どのトピックについても全体像が掴めます。**

---

[← 前へ: 11. SITL と開発ワークフロー](11_sitl_workflow.md) | [目次](README.md) | [次へ: 付録 A2. 座標系とパラメータ早見表 →](A2_frames_params.md)
