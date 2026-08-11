# 05. モード管理と安全機構

[← 前へ: 04. 制御カスケード](04_control_cascade.md) | [目次](README.md) | [次へ: 06. 自律飛行スタック →](06_autonomy_navigator.md)

---

前章で「制御チェーンがどう動くか」を見ました。この章は
**「いつ動くか、いつ止まるか、誰が決めるか」** です。
自律制御機を作るとき、ここを理解していないと
「設定値を送っているのに動かない」「勝手に RTL する」といった壁にぶつかります。

---

## 5.1 commander の役割

`src/modules/commander/` は PX4 の **司令塔** です。責務は 4 つ:

```
┌──────────────────────────────────────────────────────┐
│  commander                                            │
│                                                       │
│  ① アーミング管理    武装 / 解除、事前チェック            │
│  ② モード管理       どのフライトモードにいるか            │
│  ③ 制御モード決定    どの制御段を動かすか                 │
│  ④ フェイルセーフ    異常時に何をするか                   │
└──────────────────────────────────────────────────────┘
        │                             │
        ▼                             ▼
  vehicle_status                vehicle_control_mode
  (nav_state, arming_state)     (flag_control_*_enabled)
        │                             │
        └──────── 全モジュールが購読 ────┘
```

主要ファイル:

| ファイル | 役割 |
| --- | --- |
| `Commander.cpp` | メインループ、コマンド処理 |
| `Arming/` | アーミング状態機械 |
| `ModeManagement.cpp` | **フライトモードの登録・管理（外部モード含む）** |
| `UserModeIntention.cpp` | ユーザが意図したモードの記憶 |
| `HealthAndArmingChecks/` | **アーミング前・飛行中のチェック群** |
| `failsafe/` | **フェイルセーフ状態機械** |
| `failure_detector/` | 姿勢異常・ESC 故障などの検出 |
| `HomePosition.cpp` | ホーム位置の設定 |
| `*_calibration.cpp` | 各種キャリブレーション |

---

## 5.2 フライトモード（`nav_state`）

`msg/versioned/VehicleStatus.msg` に定義された `nav_state` が「現在のフライトモード」です。

| 値 | 定数 | モード | 説明 |
| --- | --- | --- | --- |
| 0 | `NAVIGATION_STATE_MANUAL` | Manual | 完全手動（角度指令） |
| 1 | `NAVIGATION_STATE_ALTCTL` | Altitude | 高度保持 + 手動水平 |
| 2 | `NAVIGATION_STATE_POSCTL` | Position | 位置保持 + 手動移動 |
| 3 | `NAVIGATION_STATE_AUTO_MISSION` | Mission | **ミッション自動飛行** |
| 4 | `NAVIGATION_STATE_AUTO_LOITER` | Hold | その場で待機 |
| 5 | `NAVIGATION_STATE_AUTO_RTL` | Return | 帰還 |
| 6 | `NAVIGATION_STATE_POSITION_SLOW` | Position Slow | 低速位置制御 |
| 7 | `NAVIGATION_STATE_GUIDED_COURSE` | Guided Course | 固定翼の針路保持 |
| 8 | `NAVIGATION_STATE_ALTITUDE_CRUISE` | Altitude Cruise | 高度 + 巡航 |
| 10 | `NAVIGATION_STATE_ACRO` | Acro | 角速度手動 |
| 12 | `NAVIGATION_STATE_DESCEND` | Descend | 降下のみ（位置制御なし） |
| 13 | `NAVIGATION_STATE_TERMINATION` | Termination | 飛行終了（出力停止） |
| 14 | `NAVIGATION_STATE_OFFBOARD` | **Offboard** | **外部からの設定値で飛ぶ** |
| 15 | `NAVIGATION_STATE_STAB` | Stabilized | 姿勢安定化のみ |
| 17 | `NAVIGATION_STATE_AUTO_TAKEOFF` | Takeoff | 自動離陸 |
| 18 | `NAVIGATION_STATE_AUTO_LAND` | Land | 自動着陸 |
| 19 | `NAVIGATION_STATE_AUTO_FOLLOW_TARGET` | Follow Me | 対象追従 |
| 20 | `NAVIGATION_STATE_AUTO_PRECLAND` | Precision Land | 精密着陸 |
| 21 | `NAVIGATION_STATE_ORBIT` | Orbit | 円軌道 |
| 22 | `NAVIGATION_STATE_AUTO_VTOL_TAKEOFF` | VTOL Takeoff | VTOL 離陸 |
| **23〜30** | `NAVIGATION_STATE_EXTERNAL1`〜`EXTERNAL8` | **外部モード** | **ROS 2 から登録する独自モード** |

> [!IMPORTANT]
> **23〜30 の 8 枠が、自作の自律制御モードを登録するための枠**です。
> これが [09 章](09_route_external_mode.md) の外部モードの仕組みです。

現在のモードは次で確認できます:

```sh
commander status
listener vehicle_status
```

### モードの切り替え方

| 手段 | 方法 |
| --- | --- |
| RC 送信機 | モードスイッチ（`COM_FLTMODE1`〜 で割り当て） |
| GCS | QGroundControl のモード選択 |
| MAVLink | `MAV_CMD_DO_SET_MODE` コマンド |
| ROS 2 | `VehicleCommand` トピックに `VEHICLE_CMD_DO_SET_MODE` を publish |
| 内部 | フェイルセーフによる自動遷移 |

ROS 2 から Offboard に入る例:

```
VehicleCommand.command = VEHICLE_CMD_DO_SET_MODE (176)
VehicleCommand.param1  = 1                        (カスタムモード有効)
VehicleCommand.param2  = 6                        (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
```

カスタムモード番号の定義は `src/modules/commander/px4_custom_mode.h` にあります。

---

## 5.3 `vehicle_control_mode` — 制御段の ON/OFF

commander は `nav_state` を見て `vehicle_control_mode` の各フラグを決めます。
**このフラグが、04 章の各制御段が動くかどうかを直接決定します。**

`msg/VehicleControlMode.msg`:

```
bool flag_armed
bool flag_multicopter_position_control_enabled
bool flag_control_manual_enabled
bool flag_control_auto_enabled
bool flag_control_offboard_enabled
bool flag_control_position_enabled
bool flag_control_velocity_enabled
bool flag_control_altitude_enabled
bool flag_control_climb_rate_enabled
bool flag_control_attitude_enabled
bool flag_control_rates_enabled
bool flag_control_allocation_enabled
```

モードごとのフラグの立ち方（マルチコプター、概略）:

| モード | position | velocity | altitude | climb_rate | attitude | rates |
| --- | --- | --- | --- | --- | --- | --- |
| Manual / Stabilized | – | – | – | – | ✓ | ✓ |
| Acro | – | – | – | – | – | ✓ |
| Altitude | – | – | ✓ | ✓ | ✓ | ✓ |
| Position | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Mission / Hold / RTL / Takeoff / Land | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| Offboard | 送る設定値による | | | | | |

> [!TIP]
> **デバッグの鉄則:**
> ```sh
> listener vehicle_control_mode
> ```
> 「設定値が効かない」ときは、まずこれを見てください。
> 例えば `flag_control_offboard_enabled` が false なら Offboard に入れていません。
> `flag_multicopter_position_control_enabled` が false なら `mc_pos_control` が停止しています。

Offboard の場合、`offboard_control_mode` トピックの内容から
どのフラグを立てるかが決まります（[07 章](07_route_offboard.md)）。

---

## 5.4 アーミングとヘルスチェック

### アーミング状態

`vehicle_status.arming_state`:

| 状態 | 意味 |
| --- | --- |
| `ARMING_STATE_DISARMED` | 武装解除（モータ停止） |
| `ARMING_STATE_ARMED` | 武装（モータが回りうる） |

### ヘルスチェック / アーミングチェック

`src/modules/commander/HealthAndArmingChecks/checks/` に **チェック項目ごとのファイル**があります。
これは「何がアーミングを妨げるか」の一覧そのものです。

| チェック | 内容 |
| --- | --- |
| `accelerometerCheck` / `gyroCheck` / `magnetometerCheck` / `baroCheck` | センサの存在と健全性 |
| `imuConsistencyCheck` | 複数 IMU の整合性 |
| `estimatorCheck` | **EKF2 の推定品質（位置・速度・方位の妥当性）** |
| `gnssRedundancyCheck` | GNSS の冗長性 |
| `batteryCheck` | バッテリ残量・電圧 |
| `escCheck` | ESC の状態 |
| `homePositionCheck` | ホーム位置が設定済みか |
| `missionCheck` | ミッションが妥当か |
| `geofenceCheck` | ジオフェンス設定 |
| `manualControlCheck` | RC 入力の有無 |
| `modeCheck` | 現在のモードが実行可能か |
| `offboardCheck` | **Offboard 信号が来ているか** |
| `externalChecks` | **外部モード（ROS 2）が申告するチェック** |
| `failureDetectorCheck` | 姿勢異常などの検出 |
| `cpuResourceCheck` | CPU / RAM 使用率 |
| `flightTimeCheck` | 飛行時間上限 |
| `loggerCheck` | ロガーの動作 |
| `armPermissionCheck` | 外部からのアーミング許可 |
| `companionComputerCheck` | コンパニオンコンピュータの状態 |
| `daaCheck` / `distanceSensorChecks` / `navigatorCheck` / `airspeedCheck` | 各種 |

各チェックは `checkAndReport()` で
- **アーミングを禁止する**（`armingCheckFailure`）
- **そのモードを実行不可にする**（`clearCanRunBits`）
- **フェイルセーフフラグを立てる**（`failsafeFlags()`）

のいずれかを行います。

失敗理由は GCS に表示され、シェルでも確認できます:

```sh
commander status
listener health_report
listener failsafe_flags
```

### 自律制御でよく引っかかるアーミング条件

| 症状 | 原因 | 対処 |
| --- | --- | --- |
| アームできない | GCS も RC も繋がっていない | SITL なら QGC を繋ぐ、または `COM_RC_IN_MODE` を調整 |
| Offboard に入れない | 設定値が届いていない | Offboard 移行前に `offboard_control_mode` を 2 Hz 以上で送り始める |
| 位置モードに入れない | EKF2 の位置推定が無効 | `listener estimator_status_flags`、GPS または外部Vision を確認 |
| 離陸しない | ホーム位置未設定 | `COM_HOME_EN`、位置推定の確立を待つ |

> [!WARNING]
> **SITL でチェックを外すパラメータ（`NAV_DLL_ACT 0`、`COM_ARM_WO_GPS` など）は、
> 実機では絶対に流用しないでください。** これらは安全機構そのものです。

---

## 5.5 フェイルセーフの仕組み

`src/modules/commander/failsafe/` が実装です。設計は明快です:

```
   failsafe_flags（何が起きているか）
   ┌────────────────────────────────────┐
   │ manual_control_signal_lost         │  RC 途絶
   │ gcs_connection_lost                │  データリンク途絶
   │ offboard_control_signal_lost       │  Offboard 信号途絶
   │ battery_warning / battery_unhealthy│  バッテリ
   │ geofence_breached                  │  ジオフェンス逸脱
   │ wind_limit_exceeded                │  風速超過
   │ flight_time_limit_exceeded         │  飛行時間超過
   │ position_accuracy_low              │  位置精度低下
   │ local_position_invalid / …          │  推定無効
   │ navigator_failure / mission_failure│  航法失敗
   │ vtol_fixed_wing_system_failure     │  VTOL 異常
   └────────────────────────────────────┘
                    │
                    ▼  各フラグ → 対応するパラメータ → アクション
   Action（下ほど強い。最も強いものが選ばれる）
   ┌────────────────────────────────────┐
   │ None                               │  何もしない
   │ Warn                               │  警告のみ
   │ FallbackPosCtrl / AltCtrl / Stab   │  下位モードへ降格
   │ Hold                               │  その場で待機
   │ RTL                                │  帰還
   │ Land                               │  着陸
   │ Descend                            │  降下
   │ Disarm                             │  武装解除
   │ Terminate                          │  飛行終了（出力停止）
   └────────────────────────────────────┘
```

アクションの定義は `src/modules/commander/failsafe/framework.h` の `enum class Action`、
条件からアクションへの対応は `src/modules/commander/failsafe/failsafe.cpp` の
`CHECK_FAILSAFE(...)` マクロ群にあります。

### 主要なフェイルセーフパラメータ

| パラメータ | 対象 |
| --- | --- |
| `NAV_RCL_ACT` | RC 途絶時の動作 |
| `NAV_DLL_ACT` | データリンク（GCS）途絶時の動作 |
| `COM_RC_LOSS_T` | RC 途絶と判定するまでの時間 [s] |
| `COM_DL_LOSS_T` | データリンク途絶と判定するまでの時間 [s] |
| **`COM_OF_LOSS_T`** | **Offboard 信号途絶と判定するまでの時間 [s]** |
| `COM_OBL_RC_ACT` | Offboard 途絶時の動作 |
| `COM_LOW_BAT_ACT` | バッテリ低下時の動作 |
| `COM_POS_FS_ACT` / `COM_POS_FS_EPH` | 位置推定劣化時の動作としきい値 |
| `COM_POS_LOW_ACT` / `COM_POS_LOW_EPH` | 位置精度低下時 |
| `GF_ACTION` / `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` | ジオフェンス |
| `COM_WIND_MAX` / `COM_WIND_MAX_ACT` | 風速上限 |
| `COM_FLT_TIME_MAX` / `COM_FLTT_LOW_ACT` | 飛行時間上限 |
| `COM_DISARM_LAND` | 着陸後の自動武装解除までの時間 |
| `COM_ACT_FAIL_ACT` | アクチュエータ故障時 |
| `COM_PARACHUTE` | パラシュート |

### 自律制御プログラムから見たフェイルセーフ

**フェイルセーフは味方です。** 外部プログラムがクラッシュしても、
PX4 が自動的に Hold / RTL / Land へ遷移して機体を守ります。

ただし注意点があります:

> [!WARNING]
> **フェイルセーフが発動すると、外部からの設定値は無視されます。**
> `nav_state` が変わるので `flag_control_offboard_enabled` が false になります。
> 外部プログラムは `vehicle_status` と `failsafe_flags` を購読し、
> **「自分の制御権が奪われた」ことを検出できるようにしてください。**
> 気づかずに設定値を送り続けると、原因不明の挙動として現れます。

```sh
listener failsafe_flags       # 何が起きているか
listener vehicle_status       # モードが変わっていないか
```

### フェイルセーフのシミュレーション

`src/modules/commander/failsafe/` には Emscripten でブラウザ上に
フェイルセーフ状態機械を可視化する仕組み（`emscripten.cpp`）と、
単体テスト（`failsafe_test.cpp`）があります。
**自分の設定でどう動くかを確認したいときは `failsafe_test.cpp` を読むのが最短です。**

---

## 5.6 failure_detector — 異常検出

`src/modules/commander/failure_detector/` は「機体が物理的におかしい」ことを検出します。

| 検出項目 | 内容 |
| --- | --- |
| 姿勢異常 | ロール／ピッチが規定値を超えた（`FD_FAIL_R`, `FD_FAIL_P`） |
| 外部 ATS | 外部の自動飛行終了システムからの信号 |
| ESC 異常 | ESC からのエラー報告 |
| モータ故障 | 出力と応答の不整合 |
| 対気速度異常 | 固定翼向け |

検出されると `failure_detector_status` が発行され、
フェイルセーフの `Terminate` などに繋がります。

---

## 5.7 land_detector — 着地判定

`src/modules/land_detector/` は「機体が地面にいるか」を判定します。
これは見た目より重要で、多くの制御ロジックがこれに依存しています。

`vehicle_land_detected` の主要フィールド:

| フィールド | 意味 |
| --- | --- |
| `landed` | 着地している |
| `maybe_landed` | 着地しているかもしれない |
| `ground_contact` | 地面に接触している |
| `freefall` | 自由落下中 |
| `in_ground_effect` | 地面効果の影響下 |

判定材料: 推力が低い / 速度が小さい / 姿勢が水平 / 距離計の値 など。
ヒステリシス（`src/lib/hysteresis/`）でチャタリングを防いでいます。

**使われ方の例:**
- `mc_rate_control`: 着地中は積分を更新しない（`RateControl::update()` の `landed` 引数）
- `mc_pos_control`: 離陸ランプの制御
- `commander`: 着陸後の自動武装解除（`COM_DISARM_LAND`）

> [!WARNING]
> **Offboard で低推力の飛行をすると、誤って「着地した」と判定されて空中で武装解除される
> ことがあります。** `src/modules/mc_raptor/README.md` にも同じ注意があり、
> SITL では `COM_DISARM_LAND -1` で無効化する例が示されています。
> 実機ではこの値をいじる前に、なぜ誤判定するのかを調べてください。

---

## 5.8 この章のまとめ

- **commander が司令塔**。アーミング・モード・制御モード・フェイルセーフを一手に担う
- `vehicle_status.nav_state` = 現在のフライトモード（**23〜30 が外部モード枠**）
- **`vehicle_control_mode` のフラグが各制御段の動作可否を決める**
- アーミングチェックは `HealthAndArmingChecks/checks/` に項目ごとのファイルがある
- フェイルセーフは「**フラグ → パラメータ → アクション**」の明快な設計。
  最も強いアクションが選ばれる
- **外部プログラムは `vehicle_status` と `failsafe_flags` を監視し、
  制御権を失ったことを検出できるようにする**
- `land_detector` の判定は多くの制御ロジックの前提になっている

次章では、PX4 が標準で持っている自律飛行のスタックを見ます。

---

[← 前へ: 04. 制御カスケード](04_control_cascade.md) | [目次](README.md) | [次へ: 06. 自律飛行スタック →](06_autonomy_navigator.md)
