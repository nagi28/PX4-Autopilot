# 01. アーキテクチャ全体像

[← 目次に戻る](README.md) | [次へ: 02. uORB と実行モデル →](02_uorb_runtime.md)

---

## 1.1 PX4 は 2 層でできている

PX4 は大きく **フライトスタック** と **ミドルウェア** の 2 層に分かれます。

```
┌─────────────────────────────────────────────────────────┐
│  フライトスタック (Flight Stack)                          │
│  推定・誘導・制御のアルゴリズム群                            │
│  ekf2 / mc_pos_control / mc_att_control / mc_rate_control │
│  navigator / flight_mode_manager / control_allocator      │
└─────────────────────────────────────────────────────────┘
                          ↕ uORB
┌─────────────────────────────────────────────────────────┐
│  ミドルウェア (Middleware)                                │
│  uORB メッセージバス / デバイスドライバ /                   │
│  外部通信 (mavlink, uxrce_dds_client) / ログ (logger) /    │
│  パラメータ / シミュレーション層                             │
└─────────────────────────────────────────────────────────┘
                          ↕
┌─────────────────────────────────────────────────────────┐
│  OS 抽象化層 (platforms/)                                 │
│  NuttX (実機FMU) / POSIX (SITL・Linux機) / QuRT           │
└─────────────────────────────────────────────────────────┘
```

**重要な設計思想**: フライトスタックのモジュールは互いを直接呼び出しません。
すべて uORB トピック経由の非同期メッセージパッシングです。
このおかげで、**任意のブロックを実行時に差し替えられます**。

自律制御を実装するとは、要するに
「このブロック図のどこかに自分のモジュールを差し込む」ことです。

> 公式の概念解説は `docs/en/concept/architecture.md` にあります。本書はそれをソースコードと対応づけて詳述します。

---

## 1.2 リポジトリのディレクトリ地図

初見で迷わないための地図です。**太字が最初に読むべき場所**です。

```
PX4-Autopilot/
├── src/
│   ├── modules/          ★ フライトスタックの本体。ここが主戦場
│   ├── drivers/            センサ・アクチュエータのデバイスドライバ
│   ├── lib/              ★ 再利用ライブラリ（数学、制御則、地理計算）
│   ├── examples/         ★ 自作モジュールの雛形として最重要
│   ├── templates/          モジュールテンプレート
│   ├── systemcmds/         シェルコマンド（param, listener, top, uorb …）
│   └── include/            共通ヘッダ
├── msg/                  ★ uORB メッセージ定義（.msg）。API 仕様書に相当
│   └── versioned/          ROS 2 と共有する互換性保証付きメッセージ
├── platforms/
│   ├── common/uORB/      ★ uORB の実装本体
│   ├── nuttx/              実機 FMU 用の OS 層
│   ├── posix/              SITL / Linux 機用の OS 層
│   └── ros2/               ROS 2 連携（メッセージ変換など）
├── boards/               ★ ビルドターゲット定義。どのモジュールを載せるか
│   └── px4/sitl/           SITL 用ボード定義
├── ROMFS/px4fmu_common/
│   ├── init.d/           ★ 起動スクリプト（実機）
│   ├── init.d-posix/       起動スクリプト（SITL）
│   └── init.d/airframes/   機体フレームごとの初期パラメータ
├── Tools/                  ビルド補助、シミュレーション連携、ログ解析
├── test/                   統合テスト
└── docs/                   公式ドキュメント（英語）
```

### `src/modules/` の主要モジュール

自律制御に関わるものだけ抜粋します。

| モジュール | 役割 | 本書の章 |
| --- | --- | --- |
| `sensors` | 複数センサの統合・投票・IMU 積分 | [03](03_sensing_estimation.md) |
| `ekf2` | 拡張カルマンフィルタによる姿勢・位置推定 | [03](03_sensing_estimation.md) |
| `mc_pos_control` | マルチコプター位置・速度制御 | [04](04_control_cascade.md) |
| `mc_att_control` | マルチコプター姿勢制御 | [04](04_control_cascade.md) |
| `mc_rate_control` | マルチコプター角速度制御 | [04](04_control_cascade.md) |
| `control_allocator` | 推力・トルク → 各モータ出力への配分 | [04](04_control_cascade.md) |
| `commander` | アーミング、モード管理、フェイルセーフ | [05](05_modes_safety.md) |
| `land_detector` | 着地判定 | [05](05_modes_safety.md) |
| `navigator` | ミッション、RTL、離着陸の航法 | [06](06_autonomy_navigator.md) |
| `flight_mode_manager` | FlightTask による設定値生成 | [06](06_autonomy_navigator.md) |
| `mavlink` | MAVLink 送受信（GCS、コンパニオン） | [07](07_route_offboard.md) |
| `uxrce_dds_client` | ROS 2 (DDS) 連携 | [07](07_route_offboard.md), [09](09_route_external_mode.md) |
| `logger` | ULog 記録 | [11](11_sitl_workflow.md) |

### `src/lib/` の主要ライブラリ

制御則を書くときに再利用すべきものです。**車輪の再発明を避けてください**。

| ライブラリ | 内容 |
| --- | --- |
| `matrix` | 行列・ベクトル・クォータニオン・オイラー角。ヘッダオンリー |
| `mathlib` | `constrain`, `radians`, フィルタ（LPF, notch）、スルーレート |
| `geo` | 緯度経度 ⇄ ローカル NED 変換、方位角計算 |
| `rate_control` | 角速度 PID 制御則（`mc_rate_control` が使う） |
| `pid` / `pid_design` | 汎用 PID とゲイン設計 |
| `control_allocation` | 推力配分の擬似逆行列 / 逐次デサチュレーション |
| `motion_planning` | 躍度制限付き軌道生成（`VelocitySmoothing` など） |
| `slew_rate` | 変化率制限 |
| `hysteresis` | ヒステリシス付き真偽判定（チャタリング防止） |
| `collision_prevention` | 距離センサに基づく衝突回避 |

---

## 1.3 ビルドターゲットの読み方

PX4 のビルドコマンドは `make <vendor>_<board>_<config>` の形式です。

```sh
make px4_fmu-v6x_default     # Pixhawk 6X 実機向け
make px4_sitl_default        # SITL（シミュレーション）
make px4_sitl gz_x500        # SITL + Gazebo で x500 クアッドを起動
make list_config_targets     # 選択可能なターゲット一覧
```

### ボード定義がモジュール構成を決める

「どのモジュールをファームウェアに含めるか」は `boards/<vendor>/<board>/<config>.px4board`
で決まります。Kconfig 形式です。

`boards/px4/sitl/default.px4board` の抜粋:

```
CONFIG_PLATFORM_POSIX=y
CONFIG_MODULES_COMMANDER=y
CONFIG_MODULES_CONTROL_ALLOCATOR=y
CONFIG_MODULES_EKF2=y
CONFIG_MODULES_FLIGHT_MODE_MANAGER=y
CONFIG_MODULES_MC_ATT_CONTROL=y
...
```

**自作モジュールを作ったら、ここに `CONFIG_MODULES_MY_MODULE=y` を追加しないと
ビルドに含まれません。** これは [08 章](08_route_internal.md) で詳述します。

対話的に編集する場合:

```sh
make px4_sitl_default boardconfig   # ncurses メニューが開く
```

SITL のボード定義バリエーション（`boards/px4/sitl/` 配下）:

| ファイル | 用途 |
| --- | --- |
| `default.px4board` | 標準 SITL |
| `sih.px4board` | SIH（Simulation-In-Hardware、外部シミュレータ不要） |
| `nolockstep.px4board` | ロックステップ無効（実時間動作） |
| `replay.px4board` | ログ再生（推定器のオフライン検証用） |
| `neural.px4board` / `raptor.px4board` | ニューラルネット制御の実験用 |

---

## 1.4 起動シーケンス — 電源投入から飛行可能まで

**「なぜこのモジュールが動いているのか」を理解する鍵は起動スクリプトです。**

実機のエントリポイントは `ROMFS/px4fmu_common/init.d/rcS` です。
主要な流れ（行番号は目安）:

```
rcS
 │
 ├─ ver all                        バージョン表示
 ├─ パラメータのロード                param load
 ├─ tone_alarm start               ブザー
 ├─ dataman start                  ミッション等の永続ストレージ
 ├─ send_event start               イベント通知
 ├─ load_mon start                 CPU/メモリ監視
 │
 ├─ . rc.board_sensors             ボード固有センサ起動
 ├─ . rc.sensors                   共通センサドライバ起動（パラメータで分岐）
 ├─ battery_status start
 ├─ sensors start                  ★ センサ統合モジュール
 │
 ├─ ekf2 start &                   ★ 状態推定（EKF2_EN=1 のとき）
 │
 ├─ commander start                ★ モード管理・アーミング
 ├─ dshot start / pwm_out start    アクチュエータ出力ドライバ
 │
 ├─ . rc.vehicle_setup             ★ 機体タイプ別の分岐
 │    └─ VEHICLE_TYPE=mc なら
 │         . rc.mc_defaults        マルチコプター既定パラメータ
 │         . rc.mc_apps            ★ マルチコプター制御モジュール群
 │
 ├─ mavlink start                  通信リンク
 ├─ navigator start                ★ 航法（ミッション/RTL）
 └─ . rc.logging                   ログ記録
```

### 機体タイプの分岐

`ROMFS/px4fmu_common/init.d/rc.vehicle_setup` が `$VEHICLE_TYPE` を見て分岐します。

```sh
if [ $VEHICLE_TYPE = mc ]
then
	. ${R}etc/init.d/rc.mc_defaults
	. ${R}etc/init.d/rc.mc_apps
fi
```

`$VEHICLE_TYPE` は **エアフレーム定義ファイル**が設定します。
どのエアフレーム定義が読まれるかはパラメータ `SYS_AUTOSTART` で決まり、
実体は `ROMFS/px4fmu_common/init.d/airframes/` にあります
（例: `4001_quad_x` → クアッドコプター X 配置）。

### マルチコプターの制御モジュール群

`ROMFS/px4fmu_common/init.d/rc.mc_apps` の中身がそのまま
[04 章](04_control_cascade.md) の制御チェーンに対応します:

```sh
control_allocator start           # 推力配分（最下流）
mc_rate_control start             # 角速度制御
mc_att_control start              # 姿勢制御
mc_hover_thrust_estimator start   # ホバリング推力の推定
flight_mode_manager start         # FlightTask による設定値生成
mc_pos_control start              # 位置・速度制御
land_detector start multicopter   # 着地判定
```

> [!TIP]
> 起動スクリプトの `if param compare -s XXX 1` という条件分岐に注目してください。
> **多くのモジュールはパラメータで ON/OFF できます**。
> 例えば `MC_NN_EN=1` でニューラルネット制御モジュール `mc_nn_control` が起動します。
> 自作モジュールも同じパターンで条件起動させるのが PX4 流です。

### SITL の起動シーケンス

SITL は `ROMFS/px4fmu_common/init.d-posix/rcS` から始まり、
シミュレータごとのスクリプト（`px4-rc.gzsim`, `px4-rc.jmavsim`, `px4-rc.sihsim` など）を読み込みます。
実機との違いは「センサドライバの代わりにシミュレータ接続モジュールが動く」点だけで、
**フライトスタックは実機とまったく同じコードが動きます**。これが SITL が有効な理由です。

---

## 1.5 更新レートの考え方

PX4 は「データが来たら走る」イベント駆動です。したがって
**更新レートは基本的に上流のドライバが決めます**。

| 段 | 典型的なレート | 決定要因 |
| --- | --- | --- |
| IMU ドライバ（生データ） | 1 kHz 前後 | センサの ODR 設定 |
| `vehicle_angular_velocity` | 数百 Hz〜1 kHz | `IMU_GYRO_RATEMAX` |
| `mc_rate_control` | `vehicle_angular_velocity` と同じ | コールバック駆動 |
| `vehicle_attitude`（EKF2 出力） | 250 Hz 前後 | EKF2 の出力予測器 |
| `mc_att_control` | `vehicle_attitude` と同じ | コールバック駆動 |
| `vehicle_local_position`（EKF2 出力） | 50 Hz 前後 | EKF2 の更新周期 |
| `mc_pos_control` | `vehicle_local_position` と同じ | コールバック駆動 |
| `flight_mode_manager` | 50 Hz（20 ms 間隔に制限） | `FlightModeManager::init()` |
| `navigator` | 数 Hz | 低頻度で十分 |

実機・SITL 上で実測するには `uorb top` を使います（[02 章](02_uorb_runtime.md#27-実機と-sitl-で使う調査コマンド)）。

> [!IMPORTANT]
> **自律制御を外部から行う場合、この階層構造がレイテンシ予算を決めます。**
> 角速度制御（1 kHz 級）を外部から行うのは現実的ではありません。
> 一方、位置設定値（数十 Hz）なら companion computer から十分に送れます。
> 詳細は [10 章](10_route_comparison.md) を参照してください。

---

## 1.6 この章のまとめ

- PX4 は **フライトスタック / ミドルウェア / OS 抽象化層** の 3 層構造
- モジュール間は **uORB のみ**で繋がる。だから差し替えが容易
- 「どのモジュールが動くか」は **ボード定義（`.px4board`）** と **起動スクリプト（`rc.*`）** が決める
- 起動スクリプトを読めば、システムの構成が一目で分かる
- 更新レートは上流のドライバが決める。**イベント駆動**

次章では、その uORB がどう実装され、モジュールがどう実行されるかを見ます。

---

[← 目次に戻る](README.md) | [次へ: 02. uORB と実行モデル →](02_uorb_runtime.md)
