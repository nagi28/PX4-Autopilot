# 付録 A2. 座標系とパラメータ早見表

[← 前へ: 付録 A1. 主要 uORB トピック早見表](A1_uorb_topics.md) | [目次](README.md)

---

## A2.1 座標系 ★ 最初に読むべき付録

**ROS ユーザが最も高い確率で踏む地雷が座標系です。**
PX4 のトピックに直接 publish するとき、**自動変換は一切されません**。

### PX4 と ROS の違い

| | PX4 | ROS（REP 103） |
| --- | --- | --- |
| ワールド座標 | **NED**（North-East-Down） | **ENU**（East-North-Up） |
| 機体座標 | **FRD**（Forward-Right-Down） | **FLU**（Forward-Left-Up） |
| 高度 | z が下向き正（**上昇 = z が減る**） | z が上向き正 |
| 方位の基準 | 北から時計回り | 東から反時計回り |
| クォータニオン | `(w, x, y, z)` の順 | `(x, y, z, w)` の順（ROS メッセージ内） |

### 図解

```
        PX4: NED（ワールド）              ROS: ENU（ワールド）
              N (x)                            U (z)
              ↑                                ↑
              │                                │
              │                                │
     W ───────┼───────→ E (y)         S ───────┼───────→ E (x)
              │                                │
              │                                │
              ↓ D (z, 地面方向)                 ↓ N (y)  ※ y は北、z は上

        PX4: FRD（機体）                  ROS: FLU（機体）
              F (x, 機首)                       F (x, 機首)
              ↑                                ↑
              │                                │
     ─────────┼───────→ R (y, 右)     L (y,左)←┼─────────
              │                                │
              ↓ D (z, 機体下方)                 ↑ U (z, 機体上方)
```

### よく使う値の対応

| やりたいこと | PX4 の値（NED） |
| --- | --- |
| 高度 5 m でホバリング | `position = {0, 0, -5.0}`（**z は負**） |
| 北へ 2 m/s | `velocity = {2.0, 0, 0}` |
| 東へ 2 m/s | `velocity = {0, 2.0, 0}` |
| 南へ 2 m/s | `velocity = {-2.0, 0, 0}` |
| **上昇 1 m/s** | `velocity = {0, 0, -1.0}`（**負**） |
| 下降 1 m/s | `velocity = {0, 0, 1.0}` |
| 北を向く | `yaw = 0` |
| 東を向く | `yaw = π/2` |
| 南を向く | `yaw = π`（または `-π`） |
| 西を向く | `yaw = -π/2` |

> [!WARNING]
> **「なぜか機体が地面に突っ込む」の原因はほぼこれです。**
> ROS の感覚で `position = {0, 0, 5.0}` と書くと、
> PX4 では「地下 5 m を目指す」という意味になります。

### 変換の考え方

**ENU ↔ NED**（ワールド座標）:

```
NED.x (North) =  ENU.y
NED.y (East)  =  ENU.x
NED.z (Down)  = -ENU.z
```

これは「x と y を入れ替えて z を反転」＝ **z 軸まわり 90° 回転 + x 軸まわり 180° 回転**
と等価です。**この変換は自己逆変換**（2 回適用すると元に戻る）です。

**FLU ↔ FRD**（機体座標）:

```
FRD.x (Forward) =  FLU.x
FRD.y (Right)   = -FLU.y
FRD.z (Down)    = -FLU.z
```

**方位角**:

```
yaw_NED = π/2 - yaw_ENU
```

### 実装済みのヘルパを使う

自分で書かず、既存のものを使ってください:

- [px4_ros_com](https://github.com/PX4/px4_ros_com) の `src/lib/frame_transforms.cpp`
  - `enu_to_ned_local_frame()`, `ned_to_enu_local_frame()`
  - `baselink_to_aircraft_body_frame()`（FLU → FRD）
  - `px4_to_ros_orientation()`, `ros_to_px4_orientation()`
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) を使えば
  ライブラリ側が座標系を扱ってくれます

### `VehicleOdometry` は座標系を明示できる

外部推定値を送るとき（`vehicle_visual_odometry` / `vehicle_mocap_odometry`）は、
`msg/versioned/VehicleOdometry.msg` のフィールドで座標系を宣言できます:

```
uint8 pose_frame
  POSE_FRAME_NED = 1        # 真北基準の NED
  POSE_FRAME_FRD = 2        # 任意の一定方位オフセットを持つ FRD（z は下）

uint8 velocity_frame
  VELOCITY_FRAME_NED      = 1   # 現在位置での NED
  VELOCITY_FRAME_FRD      = 2   # 現在位置での FRD
  VELOCITY_FRAME_BODY_FRD = 3   # 機体固定 FRD
```

**SLAM のように「起動位置を原点とし、真北が分からない」場合は
`POSE_FRAME_FRD` を使います。** 真北基準でないことを PX4 に伝えられます。

クォータニオンは **`(w, x, y, z)` の順、ハミルトン規約、body → world の受動回転**です。

---

## A2.2 パラメータ早見表

パラメータの完全な一覧は各モジュールの `*_params.yaml` / `module.yaml` にあります。
シェルから `param show <接頭辞>_*` で確認するのが確実です。

### 接頭辞の意味

| 接頭辞 | 担当 | 定義場所 |
| --- | --- | --- |
| `MPC_` | マルチコプター位置制御 | `src/modules/mc_pos_control/*.yaml` |
| `MC_` | マルチコプター姿勢・角速度制御 | `src/modules/mc_att_control/`, `mc_rate_control/` |
| `CA_` | 推力配分（機体形状） | `src/modules/control_allocator/module.yaml` |
| `EKF2_` | 状態推定 | `src/modules/ekf2/params_*.yaml` |
| `COM_` | commander（安全・モード） | `src/modules/commander/commander_params.yaml` |
| `NAV_` | navigator（航法・フェイルセーフ） | `src/modules/navigator/*.yaml` |
| `GF_` | ジオフェンス | `src/modules/navigator/geofence_params.yaml` |
| `IMU_` | IMU フィルタ | `src/modules/sensors/*.yaml` |
| `SENS_` | センサ有効化 | `src/modules/sensors/sensor_params.yaml` |
| `SDLOG_` | ログ記録 | `src/modules/logger/` |
| `SYS_` | システム全般 | `src/systemcmds/`, `platforms/` |
| `RTL_` | 帰還 | `src/modules/navigator/rtl_params.yaml` |
| `FD_` | 故障検出 | `src/modules/commander/failure_detector/` |

---

### 位置・速度制御（`MPC_`）

| パラメータ | 意味 |
| --- | --- |
| `MPC_XY_P` | 水平位置 P ゲイン |
| `MPC_Z_P` | 垂直位置 P ゲイン |
| `MPC_XY_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 水平速度 PID |
| `MPC_Z_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 垂直速度 PID |
| `MPC_VEL_NF_FRQ` / `MPC_VEL_NF_BW` | 速度のノッチフィルタ（既定 0 Hz = 無効 / 5 Hz） |
| `MPC_VEL_LP` | 速度のローパス遮断周波数（既定 0 Hz = 無効） |
| `MPC_VELD_LP` | **速度微分のローパス遮断周波数（既定 5 Hz、D 項のノイズ対策）** |
| `MPC_XY_VEL_MAX` | 水平速度上限 [m/s] |
| `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN` | 上昇／下降速度上限 |
| `MPC_XY_CRUISE` | 自律飛行時の巡航速度 |
| `MPC_ACC_HOR` / `MPC_ACC_HOR_MAX` | 水平加速度 |
| `MPC_ACC_UP_MAX` / `MPC_ACC_DOWN_MAX` | 垂直加速度上限 |
| `MPC_JERK_MAX` / `MPC_JERK_AUTO` | 躍度上限（平滑化の強さ） |
| `MPC_TILTMAX_AIR` | 飛行中の最大傾斜角 [deg] |
| `MPC_TILTMAX_LND` | 離着陸時の最大傾斜角 |
| `MPC_THR_HOVER` | ホバリング推力 [0-1] |
| `MPC_THR_MIN` / `MPC_THR_MAX` | 推力の下限／上限 |
| `MPC_THR_XY_MARG` | 水平推力に確保するマージン |
| `MPC_LAND_SPEED` | 着陸降下速度 |
| `MPC_TKO_SPEED` / `MPC_TKO_RAMP_T` | 離陸速度・ランプ時間 |
| `MPC_YAW_MODE` | 自律飛行時の機首方位の決め方 |
| `MPC_YAWRAUTO_MAX` | 自律時のヨーレート上限 |
| `MPC_XY_ERR_MAX` / `MPC_Z_ERR_MAX` | 位置誤差の上限 |
| `MPC_POS_MODE` | Position モードの入力方式 |
| `MPC_ACC_DECOUPLE` | 水平・垂直加速度を分離するか |

### 姿勢・角速度制御（`MC_`）

| パラメータ | 意味 |
| --- | --- |
| `MC_ROLL_P` / `MC_PITCH_P` / `MC_YAW_P` | 姿勢 P ゲイン |
| `MC_YAW_WEIGHT` | ヨーの優先度（下げるとロール/ピッチ優先） |
| `MC_ROLLRATE_MAX` / `MC_PITCHRATE_MAX` / `MC_YAWRATE_MAX` | 角速度上限 [deg/s] |
| `MC_REF_W_N` | 姿勢参照モデルの自然角周波数 |
| `MC_REF_FF` / `MC_REF_FF_MAX` | 参照モデルのフィードフォワード |
| `MC_ROLLRATE_K` / `MC_PITCHRATE_K` / `MC_YAWRATE_K` | **角速度 PID の全体ゲイン** |
| `MC_ROLLRATE_P` / `_I` / `_D` / `_FF` | ロール角速度 PID |
| `MC_PITCHRATE_P` / `_I` / `_D` / `_FF` | ピッチ角速度 PID |
| `MC_YAWRATE_P` / `_I` / `_D` / `_FF` | ヨー角速度 PID |
| `MC_RR_INT_LIM` / `MC_PR_INT_LIM` / `MC_YR_INT_LIM` | 積分上限 |
| `MC_YAW_TQ_CUTOFF` | ヨートルク出力のローパス周波数 |
| `MC_BAT_SCALE_EN` | バッテリ電圧補償 |
| `MC_AIRMODE` | 低スロットル時も姿勢制御を維持するか |
| `MC_ACRO_R_MAX` / `_P_MAX` / `_Y_MAX` | Acro モードの角速度上限 |
| `MC_AT_EN` | 姿勢オートチューンを有効化 |

### 推力配分（`CA_`）

| パラメータ | 意味 |
| --- | --- |
| `CA_AIRFRAME` | 機体タイプ |
| `CA_METHOD` | 配分アルゴリズム（0: 擬似逆行列 / 1: 逐次デサチュレーション） |
| `CA_ROTOR_COUNT` | ロータ数 |
| `CA_ROTOR{n}_PX` / `_PY` / `_PZ` | ロータ n の位置 [m] |
| `CA_ROTOR{n}_AX` / `_AY` / `_AZ` | ロータ n の推力軸方向 |
| `CA_ROTOR{n}_KM` | トルク係数（符号が回転方向） |
| `CA_FAILURE_MODE` | モータ故障時の挙動 |
| `CA_SV_CS_COUNT` | 制御面（サーボ）の数 |

### 状態推定（`EKF2_`）

| パラメータ | 意味 |
| --- | --- |
| `EKF2_EN` | EKF2 を有効化 |
| `EKF2_HGT_REF` | **高度の基準センサ**（気圧 / GNSS / 距離計 / 外部Vision） |
| `EKF2_GPS_CTRL` | GNSS の使い方（ビットマスク） |
| `EKF2_BARO_CTRL` | 気圧の使い方 |
| `EKF2_MAG_TYPE` | 磁気の使い方 |
| `EKF2_RNG_CTRL` | 距離計の使い方 |
| `EKF2_OF_CTRL` | オプティカルフローの使い方 |
| **`EKF2_EV_CTRL`** | **外部Vision の使い方（ビットマスク）** |
| `EKF2_EV_DELAY` | 外部Vision の遅延補償 [ms] |
| `EKF2_EV_POS_X` / `_Y` / `_Z` | 外部Vision センサの取り付け位置 |
| `EKF2_IMU_POS_X` / `_Y` / `_Z` | IMU の取り付け位置 |
| `EKF2_MULTI_IMU` | 複数 EKF2 インスタンスの数 |

### 安全とフェイルセーフ（`COM_`, `NAV_`, `GF_`）

| パラメータ | 意味 |
| --- | --- |
| **`COM_OF_LOSS_T`** | **Offboard 信号喪失と判定するまでの時間 [s]** |
| `COM_OBL_RC_ACT` | Offboard 喪失時の動作 |
| `COM_RC_LOSS_T` | RC 喪失判定時間 |
| `COM_DL_LOSS_T` | データリンク喪失判定時間 |
| `NAV_RCL_ACT` | RC 喪失時の動作 |
| `NAV_DLL_ACT` | データリンク喪失時の動作 |
| `COM_LOW_BAT_ACT` | バッテリ低下時の動作 |
| `COM_POS_FS_ACT` / `COM_POS_FS_EPH` | 位置推定劣化時の動作としきい値 |
| `COM_POS_LOW_ACT` / `COM_POS_LOW_EPH` | 位置精度低下時 |
| `COM_DISARM_LAND` | 着陸後の自動武装解除までの時間（-1 で無効） |
| `COM_DISARM_PRFLT` | 離陸前の自動武装解除時間 |
| `COM_ARM_WO_GPS` | GPS なしでアームを許可するか |
| `COM_RC_IN_MODE` | RC 入力の扱い |
| `COM_FLT_TIME_MAX` / `COM_FLTT_LOW_ACT` | 飛行時間上限とその動作 |
| `COM_WIND_MAX` / `COM_WIND_MAX_ACT` | 風速上限とその動作 |
| `COM_ACT_FAIL_ACT` | アクチュエータ故障時 |
| `COM_HOME_EN` | ホーム位置の自動設定 |
| `COM_SPOOLUP_TIME` | アーム後のモータ立ち上げ時間 |
| `GF_ACTION` | **ジオフェンス逸脱時の動作** |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` | ジオフェンスの水平／垂直距離 |
| `GF_SOURCE` | ジオフェンス判定に使う位置 |
| `NAV_ACC_RAD` | ウェイポイント到達判定半径 |
| `NAV_MC_ALT_RAD` | 高度の到達判定 |
| `NAV_LOITER_RAD` | 旋回半径 |
| `NAV_MIN_LTR_ALT` | 最低旋回高度 |
| `RTL_RETURN_ALT` / `RTL_DESCEND_ALT` | RTL の帰還高度・降下高度 |
| `RTL_TYPE` | RTL の方式 |
| `FD_FAIL_R` / `FD_FAIL_P` | 姿勢異常と判定する角度 |

### センサ・フィルタ（`IMU_`）

| パラメータ | 意味 |
| --- | --- |
| `IMU_GYRO_RATEMAX` | ジャイロの発行レート上限（= 角速度制御の周期） |
| `IMU_GYRO_CUTOFF` | ジャイロのローパス周波数 |
| `IMU_DGYRO_CUTOFF` | 角加速度（D 項用）のローパス周波数 |
| `IMU_GYRO_NF0_FRQ` / `_BW` | 静的ノッチフィルタ |
| `IMU_GYRO_DNF_EN` | 動的ノッチフィルタ |
| `IMU_ACCEL_CUTOFF` | 加速度計のローパス周波数 |

### 出力（`PWM_MAIN_` / `PWM_AUX_`）

チャンネルごとに添字が付きます（`PWM_MAIN_MIN1`, `PWM_MAIN_MIN2`, …）。
`src/drivers/pwm_out/module.yaml` の `standard_params` から自動生成されます。

| パラメータ | 意味 |
| --- | --- |
| `PWM_MAIN_FUNC{n}` | チャンネル n に割り当てる機能（Motor 1、Servo 1 など） |
| `PWM_MAIN_MIN{n}` / `PWM_MAIN_MAX{n}` | パルス幅の下限／上限 [µs] |
| `PWM_MAIN_CENT{n}` | 中央値（サーボ用） |
| `PWM_MAIN_DIS{n}` | 武装解除時の出力値 |
| `PWM_MAIN_FAIL{n}` | フェイルセーフ時の出力値 |
| `PWM_MAIN_TIM{t}` | 出力プロトコル（PWM / OneShot / DShot / BDShot）。**添字はタイマ番号**（チャンネルではない） |
| `PWM_MAIN_REV` | 出力の反転（ビットマスク、チャンネルごとのビット） |
| `THR_MDL_FAC` | 制御信号 → 推力の非線形モデル係数（`src/lib/mixer_module/motor_params.yaml`） |

### ログ（`SDLOG_`）

| パラメータ | 意味 |
| --- | --- |
| `SDLOG_MODE` | いつ記録するか |
| `SDLOG_PROFILE` | **記録するトピックのプロファイル（ビットマスク）** |
| `SDLOG_MAX_SIZE` | 1 ファイルの最大サイズ |
| `SDLOG_DIRS_MAX` | 保持するログ数 |

---

## A2.3 パラメータの調べ方

```sh
# シェルで確認（最も確実）
param show MPC_*
param show -c              # 既定値から変更されているものだけ
param status

# ソースから定義を探す
grep -rn "MPC_XY_P" src/modules/mc_pos_control/*.yaml
grep -rln "COM_OF_LOSS_T" src/

# YAML の定義を読む（説明・範囲・既定値が書いてある）
cat src/modules/mc_pos_control/multicopter_position_control_gain_params.yaml
```

**QGroundControl のパラメータ画面**には説明と範囲が表示され、検索もできます。
実機のチューニングではこちらが便利です。

---

## A2.4 単位の約束

| 量 | PX4 内部の単位 | パラメータの単位 |
| --- | --- | --- |
| 長さ | m | m |
| 速度 | m/s | m/s |
| 加速度 | m/s² | m/s² |
| 角度 | **rad** | **deg**（パラメータは度が多い） |
| 角速度 | **rad/s** | **deg/s** |
| 時間 | **µs**（`hrt_absolute_time()`） | s |
| 推力・トルク | 正規化 `[-1, 1]` | 正規化 |
| 気圧 | Pa | — |
| 電圧・電流 | V, A | V, A |

> [!WARNING]
> **パラメータは度、内部は弧度法**という食い違いに注意してください。
> ソースコードでは `math::radians(_param_mc_acro_r_max.get())` のように
> 読み込み時に変換しています。

タイムスタンプは `hrt_absolute_time()`（起動からのマイクロ秒）です。
ROS 2 から送るときは時刻同期（`timesync_status`）に基づいて合わせてください。

---

## A2.5 参考リンク

| リソース | URL / パス |
| --- | --- |
| PX4 公式ドキュメント | https://docs.px4.io/ |
| リポジトリ内の英語ドキュメント | `docs/en/` |
| パラメータリファレンス | https://docs.px4.io/main/en/advanced_config/parameter_reference.html |
| uORB メッセージ定義 | `msg/`, `msg/versioned/` |
| ROS 2 メッセージ | https://github.com/PX4/px4_msgs |
| ROS 2 サンプル・座標変換 | https://github.com/PX4/px4_ros_com |
| 外部モードライブラリ | https://github.com/Auterion/px4-ros2-interface-lib |
| Gazebo モデル | https://github.com/PX4/PX4-gazebo-models |
| ログ解析（Flight Review） | https://review.px4.io/ |
| ULog 解析（Python） | https://github.com/PX4/pyulog |

---

[← 前へ: 付録 A1. 主要 uORB トピック早見表](A1_uorb_topics.md) | [目次](README.md)
