# 03. センシングと状態推定

[← 前へ: 02. uORB と実行モデル](02_uorb_runtime.md) | [目次](README.md) | [次へ: 04. 制御カスケード →](04_control_cascade.md)

---

自律制御の前提は「**機体が今どこにいて、どちらを向いていて、どう動いているか**」を知ることです。
この章では生センサ値がどう処理され、EKF2 でどう融合され、
制御が使う `vehicle_attitude` / `vehicle_local_position` になるかを追います。

---

## 3.1 推定パイプライン全体図

```
 物理センサ (SPI/I2C/UART)
      │
      ▼
┌──────────────────────────────────────────────┐
│ ドライバ (src/drivers/imu/…, src/drivers/gnss/…) │
│  publish: sensor_accel, sensor_gyro,          │
│           sensor_mag, sensor_baro, sensor_gps │
└──────────────────────────────────────────────┘
      │
      ▼
┌──────────────────────────────────────────────┐
│ sensors モジュール (src/modules/sensors/)      │
│  ・キャリブレーション適用・回転補正             │
│  ・多重センサの投票／フェイルオーバ              │
│  ・IMU 積分（コーニング/スカリング補正）         │
│                                               │
│  vehicle_imu           ← IMU 積分値（Δv, Δθ）  │
│  vehicle_angular_velocity ← ★ 制御用ジャイロ     │
│  vehicle_acceleration                          │
│  vehicle_air_data      ← 気圧高度               │
│  vehicle_magnetometer                          │
│  vehicle_gps_position                          │
│  sensor_combined       ← ログ／互換用まとめ      │
└──────────────────────────────────────────────┘
      │
      ▼
┌──────────────────────────────────────────────┐
│ ekf2 (src/modules/ekf2/)                      │
│  24 誤差状態の拡張カルマンフィルタ               │
│  IMU で予測 → 各種センサで補正                  │
│                                               │
│  vehicle_attitude        ← ★ 姿勢（クォータニオン）│
│  vehicle_local_position  ← ★ ローカル NED 位置速度 │
│  vehicle_global_position ← 緯度経度高度          │
│  vehicle_odometry        ← 外部連携用            │
│  estimator_status(_flags) ← 健全性              │
└──────────────────────────────────────────────┘
      │
      ▼
   制御カスケードへ（04 章）
```

**重要**: 制御に使われるのは `vehicle_angular_velocity`（角速度制御用）と
`vehicle_attitude` / `vehicle_local_position`（姿勢・位置制御用）の 3 つです。
生センサ値（`sensor_gyro` など）は制御には使いません。

---

## 3.2 `sensors` モジュールの仕事

`src/modules/sensors/` は「生センサ → 制御が使える形」への橋渡しをします。

### サブモジュール構成

| ディレクトリ | 出力トピック | 役割 |
| --- | --- | --- |
| `vehicle_imu/` | `vehicle_imu`, `vehicle_imu_status` | ジャイロ・加速度の積分（Δθ, Δv） |
| `vehicle_angular_velocity/` | `vehicle_angular_velocity` | **制御用の角速度**（フィルタ済み） |
| `vehicle_acceleration/` | `vehicle_acceleration` | 制御用の加速度 |
| `vehicle_air_data/` | `vehicle_air_data` | 気圧高度・温度 |
| `vehicle_magnetometer/` | `vehicle_magnetometer` | 磁気（複数センサ統合） |
| `vehicle_gps_position/` | `vehicle_gps_position` | GNSS（複数受信機からブレンド） |
| `vehicle_optical_flow/` | `vehicle_optical_flow` | オプティカルフロー |
| `data_validator/` | — | センサ投票・異常検知の共通ロジック |

### `vehicle_angular_velocity` が特別な理由

`src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` は
制御ループの品質を直接決める重要モジュールです。処理内容:

1. 選択中のジャイロ（`sensor_selection` で決まる）を購読
2. EKF2 が推定したジャイロバイアス（`estimator_sensor_bias`）を減算
3. **ノッチフィルタ** — プロペラ回転由来の振動を除去
   - 静的ノッチ（`IMU_GYRO_NF0_FRQ` 等）
   - 動的ノッチ — ESC の回転数（`esc_status`）や FFT 解析（`sensor_gyro_fft`）に追従
4. **ローパスフィルタ**（`IMU_GYRO_CUTOFF`）
5. 角加速度の算出（D 項に使う）
6. `vehicle_angular_velocity` として発行 → **`mc_rate_control` を駆動**

> [!IMPORTANT]
> **機体が振動する / モータが熱くなる場合、まずここのフィルタ設定を疑ってください。**
> ジャイロノイズが D 項で増幅され、モータが高周波で振動します。
> 関連パラメータ: `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_GYRO_NF0_*`, `IMU_GYRO_DNF_EN`

### センサの多重化とフェイルオーバ

IMU が複数ある機体では、`sensors` モジュールが投票（voting）を行い、
異常なセンサを除外します（`voted_sensors_update.cpp`）。
どれが選ばれているかは `sensor_selection` トピックで分かります。

```sh
listener sensor_selection
listener vehicle_imu_status -i 0
```

さらに EKF2 自体も複数インスタンス動作でき（`EKF2_MULTI_IMU`）、
`EKF2Selector`（`src/modules/ekf2/EKF2Selector.cpp`）が最良の推定器を選びます。

---

## 3.3 EKF2 — 状態推定の中核

`src/modules/ekf2/` が PX4 の標準推定器です。
`EKF2.cpp` が uORB との入出力を担当し、`EKF/` 配下がフィルタ本体です。

### 状態ベクトル

`src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h` に定義があります。

| 状態 | 次元 | 内容 |
| --- | --- | --- |
| `quat_nominal` | 4 | 姿勢クォータニオン（NED → body） |
| `vel` | 3 | NED 速度 [m/s] |
| `pos` | 3 | NED 位置 [m] |
| `gyro_bias` | 3 | ジャイロバイアス [rad/s] |
| `accel_bias` | 3 | 加速度計バイアス [m/s²] |
| `mag_I` | 3 | 地球磁場（NED） |
| `mag_B` | 3 | 機体磁場バイアス |
| `wind_vel` | 2 | 風速（水平） |

誤差状態は 24 次元（クォータニオンが 3 次元の回転誤差になるため）。

> [!NOTE]
> **バイアスが状態に含まれている**のが重要です。
> だから PX4 は起動後しばらく静止させるとジャイロバイアスが収束し、姿勢が安定します。
> 「起動直後に動かすとドリフトする」のはこのためです。

### 予測と補正

```
   IMU (vehicle_imu の Δθ, Δv)
        │
        ▼
   [ 予測 ] 状態を積分し、共分散を伝播      ← 高レート（200〜1000 Hz）
        │
        ▼
   [ 補正 ] 各センサの観測で状態を更新       ← センサごとに異なるレート
        │        GNSS / 気圧 / 磁気 / 距離計 / フロー / 外部Vision / 対気速度 …
        ▼
   [ 出力予測器 ] 遅延補償して現在時刻の値を出す
        │
        ▼
   vehicle_attitude / vehicle_local_position / vehicle_global_position
```

**遅延補償**が PX4 EKF2 の特徴です。
GNSS のような遅延の大きいセンサに合わせて融合時刻を過去にずらし、
`output_predictor`（`src/modules/ekf2/EKF/output_predictor/`）が
「今この瞬間」の値を高レートで外挿します。
だから制御は遅延の小さい `vehicle_attitude` を使えます。

### 補正源（aid sources）

`src/modules/ekf2/EKF/aid_sources/` の各ディレクトリが 1 つの補正源に対応します。

| ディレクトリ | 補正源 | 主なパラメータ |
| --- | --- | --- |
| `gnss/` | GNSS 位置・速度・方位 | `EKF2_GPS_CTRL` |
| `barometer/` | 気圧高度 | `EKF2_BARO_CTRL` |
| `magnetometer/` | 磁気（方位） | `EKF2_MAG_TYPE` |
| `range_finder/` | レーザ・超音波距離計 | `EKF2_RNG_CTRL` |
| `optical_flow/` | オプティカルフロー | `EKF2_OF_CTRL` |
| `external_vision/` | **外部 Vision / モーキャプ** | `EKF2_EV_CTRL` |
| `aux_global_position/` | 補助的な絶対位置 | — |
| `auxvel/` | 補助速度 | — |
| `airspeed/`, `sideslip/`, `drag/` | 固定翼向け | — |
| `ranging_beacon/` | UWB 等の測距ビーコン | — |
| `gravity/` | 重力ベクトル（低速時の姿勢補正） | — |

`ZeroVelocityUpdate` / `ZeroGyroUpdate` は「静止していると判定したときに
速度 0・角速度 0 を仮想観測として与える」テクニックで、地上でのドリフトを抑えます。

### 高度基準の選択

`EKF2_HGT_REF` で「どのセンサを高度の基準にするか」を選びます
（気圧 / GNSS / 距離計 / 外部Vision）。**屋内飛行では距離計や外部Vision、
屋外では気圧か GNSS** が典型です。

> [!WARNING]
> 高度基準の選択ミスは墜落に直結します。
> 屋内で気圧基準のままドアの開閉による気圧変動を拾い、機体が上下する事故は典型例です。

---

## 3.4 外部の推定値を PX4 に入れる ★ 自律制御で最重要

GPS の効かない屋内や、SLAM・モーションキャプチャを使う場合、
**外部で計算した位置・姿勢を EKF2 に流し込む**ことができます。
これは自律制御機を作るうえで極めて重要な経路です。

### 入力トピック

| トピック | 用途 | 型 |
| --- | --- | --- |
| `vehicle_visual_odometry` | VIO / SLAM の出力 | `VehicleOdometry` |
| `vehicle_mocap_odometry` | モーションキャプチャ | `VehicleOdometry` |

EKF2 側の購読は `src/modules/ekf2/EKF2.hpp` の `_ev_odom_sub` です。

### 送り方

**ROS 2 の場合**（`src/modules/uxrce_dds_client/dds_topics.yaml` に定義済み）:

```
/fmu/in/vehicle_visual_odometry   → px4_msgs::msg::VehicleOdometry
/fmu/in/vehicle_mocap_odometry    → px4_msgs::msg::VehicleOdometry
```

**MAVLink の場合**: `ODOMETRY` / `VISION_POSITION_ESTIMATE` /
`ATT_POS_MOCAP` メッセージ（`src/modules/mavlink/mavlink_receiver.cpp` で処理）。

### 有効化パラメータ

| パラメータ | 意味 |
| --- | --- |
| `EKF2_EV_CTRL` | どの外部Vision情報を使うか（水平位置 / 垂直位置 / 速度 / 方位のビットマスク） |
| `EKF2_HGT_REF` | 高度基準を外部Visionにする場合はここを設定 |
| `EKF2_EV_DELAY` | 外部Vision の遅延補償 [ms] |
| `EKF2_EV_POS_X/Y/Z` | センサ取り付け位置のオフセット |

> [!IMPORTANT]
> **座標系に注意してください。** `VehicleOdometry` は PX4 の慣習に従います
> （位置は NED または FRD、姿勢は NED→body のクォータニオン）。
> ROS の標準は ENU / FLU なので、**必ず変換が必要**です。
> 変換については [付録 A2](A2_frames_params.md) を参照してください。
> `pose_frame` / `velocity_frame` フィールドで座標系を明示できます。

### 遅延の影響

外部推定値は必ず遅延します。`EKF2_EV_DELAY` を正しく設定しないと、
EKF2 が「過去の観測を現在の観測として」融合してしまい、位置が振動します。
**タイムスタンプを正しく打つのが最も重要**です（PX4 の `timestamp_sample` に
観測時刻を、`timestamp` に送信時刻を入れる）。

時刻同期には `timesync_status`（uXRCE-DDS が自動で行う）を使います。

---

## 3.5 推定の健全性を確認する

自律制御では「推定が信用できるか」の判定が安全に直結します。

### `vehicle_local_position` の妥当性フラグ

`msg/versioned/VehicleLocalPosition.msg` に含まれるフラグ:

| フィールド | 意味 |
| --- | --- |
| `xy_valid` / `z_valid` | 水平／垂直位置が有効 |
| `v_xy_valid` / `v_z_valid` | 水平／垂直速度が有効 |
| `xy_global` / `z_global` | 緯度経度／標高への変換基準がある |
| `dead_reckoning` | 推測航法中（絶対位置補正なし = 徐々にずれる） |
| `eph` / `epv` | 水平／垂直位置の標準偏差 [m] |
| `evh` / `evv` | 水平／垂直速度の標準偏差 [m/s] |
| `heading_good_for_control` | 方位が制御に使える精度か |

> [!TIP]
> **自律制御プログラムは、設定値を送る前にこれらのフラグを確認すべきです。**
> 特に `xy_valid` が false のときに位置設定値を送っても制御されません。
> Offboard では PX4 側もチェックしています（[07 章](07_route_offboard.md)）。

### リセットカウンタ

`xy_reset_counter`, `z_reset_counter`, `heading_reset_counter` は
**推定値がジャンプしたとき**に増加します。同時に `delta_xy`, `delta_z`, `delta_heading` に
ジャンプ量が入ります。

外部で位置制御ループを組んでいる場合、
**このカウンタの変化を検出して自分の設定値も同じだけシフトさせる**必要があります。
そうしないと、EKF がリセットした瞬間に機体が急に動きます。

PX4 内部でも同じ処理をしています
（`mc_pos_control` の Takeoff や `flight_mode_manager` の FlightTask 内）。

### 診断トピック

```sh
listener estimator_status          # 革新量（innovation）とテスト比
listener estimator_status_flags    # どの補正源が有効かのビットフラグ
listener estimator_innovations     # 各センサの革新量
listener estimator_sensor_bias     # 推定されたバイアス
```

**革新量（innovation）** = 「観測値 − 予測値」。
これが大きいまま続くなら、センサかモデルのどちらかが間違っています。
`estimator_status` の `*_test_ratio` が 1.0 を超えるとそのセンサは棄却されます。

---

## 3.6 代替の推定器

| モジュール | 用途 |
| --- | --- |
| `ekf2` | **標準**。ほぼ常にこれを使う |
| `attitude_estimator_q` | 姿勢のみの軽量推定（`ATT_EN=1`）。位置が不要な機体向け |
| `local_position_estimator` | 旧世代の位置推定（`LPE_EN=1`）。非推奨 |

起動時の分岐は `ROMFS/px4fmu_common/init.d/rcS` にあります
（`EKF2_EN` / `ATT_EN` / `LPE_EN` で切り替え）。

外部で INS を持っている場合は `external_ins_local_position` /
`external_ins_attitude` というトピック別名も用意されています
（`msg/versioned/VehicleLocalPosition.msg` の `# TOPICS` 行を参照）。

---

## 3.7 この章のまとめ

- **生センサ → `sensors` モジュール → EKF2 → 制御用トピック** の 3 段構成
- 制御が使うのは `vehicle_angular_velocity` / `vehicle_attitude` / `vehicle_local_position`
- `vehicle_angular_velocity` のフィルタ設定が制御品質を大きく左右する
- EKF2 は姿勢・速度・位置に加え **センサバイアスと風速も推定**する
- 遅延補償と出力予測器のおかげで、低遅延の姿勢・位置が得られる
- **外部推定値は `vehicle_visual_odometry` / `vehicle_mocap_odometry` から注入**（`EKF2_EV_CTRL`）
- 自律制御では `*_valid` フラグと **リセットカウンタ**を必ず監視すること

次章はいよいよ制御の本体です。

---

[← 前へ: 02. uORB と実行モデル](02_uorb_runtime.md) | [目次](README.md) | [次へ: 04. 制御カスケード →](04_control_cascade.md)
