# 09. ルート C: px4_ros2 外部モード

[← 前へ: 08. ルート B: PX4 内部に実装](08_route_internal.md) | [目次](README.md) | [次へ: 10. 3 ルートの比較と選定 →](10_route_comparison.md)

---

**ROS 2 側に書いたコードを、PX4 の「フライトモード」として登録する**方式です。

Offboard との決定的な違いは、**PX4 があなたのモードを「自分のモードの一つ」として扱う**点です:

- GCS のモード一覧に自分のモード名が出る
- **アーミングチェックに自分の条件を登録できる**
- フェイルセーフと正しく統合される
- 既存の PX4 モード（例: RTL）を自作版で**置き換え**られ、失敗時は元のモードにフォールバックする

PX4 v1.15 以降で利用可能で、公式には
[PX4 ROS 2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib) を使います。

---

## 9.1 仕組み — 8 つの外部モード枠

```
   ROS 2 側                              PX4 側
┌──────────────────┐              ┌────────────────────────────┐
│ px4_ros2::ModeBase│              │ commander/ModeManagement    │
│  を継承したクラス   │              │                            │
│                  │              │  NAVIGATION_STATE_EXTERNAL1 │ ← 23
│  ① 登録要求 ─────────────────────>│  NAVIGATION_STATE_EXTERNAL2 │ ← 24
│                  │              │  …                          │
│  ② モード ID 割当 <─────────────────│  NAVIGATION_STATE_EXTERNAL8 │ ← 30
│                  │              └────────────────────────────┘
│  ③ アーミングチェック応答 ──────────>│  HealthAndArmingChecks/     │
│                  │              │     checks/externalChecks   │
│                  │              └────────────────────────────┘
│  ④ 設定値タイプ宣言 ──────────────>│  SetpointConfig            │
│                  │              └────────────────────────────┘
│  ⑤ 設定値を送信 ─────────────────>│  trajectory_setpoint 等     │
│                  │              └────────────────────────────┘
│  ⑥ 完了通知 ────────────────────>│  ModeCompleted             │
└──────────────────┘              └────────────────────────────┘
```

枠の数は `src/modules/commander/ModeManagement.hpp` で定義されています:

```cpp
static constexpr uint8_t FIRST_EXTERNAL_NAV_STATE = vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;
static constexpr uint8_t LAST_EXTERNAL_NAV_STATE  = vehicle_status_s::NAVIGATION_STATE_EXTERNAL8;
static constexpr int MAX_NUM = LAST_EXTERNAL_NAV_STATE - FIRST_EXTERNAL_NAV_STATE + 1;   // = 8
```

**最大 8 個の外部モードを同時に登録できます。**
アーミングチェックの登録枠も 8 個です
（`src/modules/commander/HealthAndArmingChecks/checks/externalChecks.hpp` の
`MAX_NUM_REGISTRATIONS = 8`）。

---

## 9.2 登録プロトコル

### `RegisterExtComponentRequest` — 登録要求

`msg/versioned/RegisterExtComponentRequest.msg`:

```
uint64 request_id                  # ランダムな値を設定
char[25] name                      # モード名（GCS に表示される）

uint16 LATEST_PX4_ROS2_API_VERSION = 2
uint16 px4_ros2_api_version        # 上記の値を設定

bool register_arming_check         # アーミングチェックを登録するか
bool register_mode                 # モードを登録するか（arming_check も必須）
bool register_mode_executor        # モードエグゼキュータを登録するか

bool enable_replace_internal_mode  # PX4 内部モードを置き換えるか
uint8 replace_internal_mode        # 置き換える対象（NAVIGATION_STATE_*）
bool activate_mode_immediately     # 登録後すぐ有効化（エグゼキュータと併用）
bool not_user_selectable           # ユーザが選択できないモードにする
bool request_offboard_setpoints    # MAVLink 経由の Offboard 設定値を受け取るか
```

処理は `src/modules/commander/ModeManagement.cpp` の
`Modes::addExternalMode()` が行い、空いている枠を割り当てます。

### `RegisterExtComponentReply` — 応答

```
bool success
int8 arming_check_id      # アーミングチェックの登録 ID（-1 なら無効）
int8 mode_id              # 割り当てられたモード ID（= nav_state, -1 なら無効）
int8 mode_executor_id     # モードエグゼキュータ ID
```

**`mode_id` が `nav_state` の値**（23〜30）です。
これが GCS に「カスタムモード」として見えます。

### モードの置き換え（Mode Replacement）★

```
enable_replace_internal_mode = true
replace_internal_mode = NAVIGATION_STATE_AUTO_RTL
```

これで **「RTL を自分の実装で置き換える」** ことができます。
運用者が RTL を選ぶと、あなたのモードが動きます。

> [!TIP]
> **この仕組みが強力なのは、フォールバックがあるからです。**
> あなたのモードが応答しなくなったり、アーミングチェックで失敗したりすると、
> PX4 は自動的に**元の内部モードに戻します**。
> 「賢い RTL を試したいが、失敗したら標準 RTL に戻ってほしい」が実現できます。

---

## 9.3 アーミングチェックへの参加 ★ 最大の利点

PX4 は定期的に `ArmingCheckRequest` をブロードキャストし、
登録済みの外部コンポーネントは `ArmingCheckReply` で応答します。

### `ArmingCheckReply` の中身

```
uint8 registration_id             # 登録時にもらった ID
bool  can_arm_and_run             # このモードでアーム／モード切替できるか
uint8 num_events
Event[5] events                   # 失敗理由（GCS に転送される）

# このモードが必要とする推定値・条件
bool mode_req_angular_velocity        # 角速度推定が必要
bool mode_req_attitude                # 姿勢推定が必要
bool mode_req_local_alt               # ローカル高度が必要
bool mode_req_local_position          # ローカル位置が必要
bool mode_req_local_position_relaxed
bool mode_req_global_position         # グローバル位置が必要
bool mode_req_global_position_relaxed
bool mode_req_mission                 # ミッションが必要
bool mode_req_home_position           # ホーム位置が必要
bool mode_req_prevent_arming          # アームを禁止する（Land モードなど）
bool mode_req_manual_control          # 手動操作が必要
```

**これが Offboard との決定的な差です。**

- 「自分のモードは GPS が必要」と宣言すれば、
  **GPS がないときは PX4 がそのモードへの切り替えを拒否**してくれる
- 自分のプログラム内部の問題（経路計画が失敗、カメラが死んだ等）を
  `can_arm_and_run = false` + `events` で **GCS に理由付きで表示**できる

```
「自作の配送モードに入れません: マップが読み込まれていません」
                            ↑ こういうメッセージを GCS に出せる
```

### 応答しないと切り離される

`ArmingCheckRequest` は **アーム中も定期的に送られます**。
応答が止まると PX4 はそのモードを「応答なし」と判定し、
フェイルセーフを発動します。**外部プログラムのクラッシュを PX4 が検知できる**わけです。

---

## 9.4 設定値タイプの宣言

`msg/versioned/SetpointConfig.msg` で「どの種類の設定値を送るか」を宣言します。

| 定数 | 送るトピック |
| --- | --- |
| `TYPE_MULTICOPTER_GOTO` | `GotoSetpoint` |
| `TYPE_TRAJECTORY` | `TrajectorySetpoint` |
| `TYPE_TRAJECTORY_6DOF` | `TrajectorySetpoint6dof` |
| `TYPE_ATTITUDE` | `VehicleAttitudeSetpoint` |
| `TYPE_RATES` | `VehicleRatesSetpoint` |
| `TYPE_THRUST_AND_TORQUE` | `VehicleThrustSetpoint` + `VehicleTorqueSetpoint` |
| `TYPE_DIRECT_ACTUATORS` | `ActuatorMotors` + `ActuatorServos` |
| `TYPE_POSITION_TRIPLET` | `PositionSetpointTriplet` |
| `TYPE_FIXEDWING_LATERAL_LONGITUDINAL` | 固定翼向け |
| `TYPE_ROVER_*` | Rover 向け |

さらに重要なフィールド:

```
bool should_apply     # true: 現在の設定として適用 / false: 使えるかの問い合わせのみ
uint16 timeout_ms     # この時間設定値が来なければフェイルセーフ。0 で無効
```

> [!NOTE]
> **`should_apply = false` で「この設定値タイプが今の機体構成で使えるか」を
> 事前に問い合わせられます。** 応答は `SetpointConfigReply` で返ります。
> Offboard にはこの機能がありません。

commander 側では、宣言された設定値タイプから `vehicle_control_mode` のフラグが決まります
（`src/modules/commander/ModeUtil/control_mode.cpp`）:

```cpp
case vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 ... vehicle_status_s::NAVIGATION_STATE_EXTERNAL8:
    getControlMode(external_mode_setpoint_type, vehicle_control_mode);
    break;
```

---

## 9.5 モードエグゼキュータ（Mode Executor）

**モードを跨いだシーケンスを組む**ための仕組みです。

```
モード          = 「機体をどう動かすか」（設定値を送る）
モードエグゼキュータ = 「どのモードをいつ使うか」（モードを切り替える）
```

例: 自動配送ミッション

```
① Takeoff（PX4 内部モード）
      ↓ ModeCompleted を待つ
② 自作の「配送地点へ移動」モード
      ↓ 到着
③ 自作の「ペイロード投下」モード
      ↓ 完了
④ RTL（PX4 内部モード）
```

エグゼキュータは `vehicle_command_mode_executor` トピックを通じて
**PX4 の内部モードも起動できます**。

### `ModeCompleted` — 完了通知

`msg/versioned/ModeCompleted.msg`:

```
uint8 RESULT_SUCCESS = 0
uint8 RESULT_FAILURE_OTHER = 100

uint8 result       # RESULT_* のいずれか
uint8 nav_state    # どのモードの完了か
```

自作モードが「タスク完了」を通知すると、エグゼキュータが次のモードへ進みます。

### `ConfigOverrides` — 挙動の一時的な上書き

`msg/versioned/ConfigOverrides.msg`:

```
bool disable_auto_disarm         # 着陸後の自動武装解除を止める
bool defer_failsafes             # フェイルセーフを一時的に保留する
int16 defer_failsafes_timeout_s  # 保留の最大時間（0=システム既定, -1=無制限）
bool disable_auto_set_home       # ホーム位置の自動設定を止める
```

> [!WARNING]
> **`defer_failsafes` は危険な機能です。**
> 「着陸直前だけはバッテリ警告で RTL に切り替わってほしくない」といった
> 明確な理由がある場合にだけ、短いタイムアウトを付けて使ってください。

---

## 9.6 ROS 2 側の実装イメージ

[px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) を使います。
上記のプロトコルはすべてライブラリが隠蔽してくれます。

```cpp
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>

class MyDeliveryMode : public px4_ros2::ModeBase
{
public:
    explicit MyDeliveryMode(rclcpp::Node &node)
        : ModeBase(node, Settings{"Delivery"})     // ← GCS に出るモード名
    {
        // 使う設定値タイプを宣言（SetpointConfig に対応）
        _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

        // 使う状態推定を宣言（mode_req_* に対応）
        _local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    }

    // アーミングチェック: 自分の条件を PX4 に伝える
    void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter &reporter) override
    {
        if (!map_loaded_) {
            reporter.armingCheckFailureExt(events::ID("delivery_no_map"),
                                           events::Log::Error, "Map not loaded");
        }
    }

    void onActivate() override   { /* モード開始時 */ }
    void onDeactivate() override { /* モード終了時 */ }

    // 定期的に呼ばれる。ここで設定値を送る
    void updateSetpoint(float dt_s) override
    {
        const auto position = _local_position->positionNed();

        // 目標位置へ向かう設定値を送る
        _trajectory_setpoint->updatePosition(target_ned_);

        if (reachedTarget(position)) {
            completed(px4_ros2::Result::Success);   // ← ModeCompleted を送る
        }
    }

private:
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
    bool map_loaded_{false};
    Eigen::Vector3f target_ned_;
};
```

Python バインディングもあります（対応クラスは限定的）。

### PX4 内部モードを置き換える例

```cpp
ModeBase::Settings{"MyRTL", /* replace_internal_mode = */
                   ModeBase::kModeIDRtl}
```

これで RTL が自作版に置き換わり、失敗時は標準 RTL にフォールバックします。

---

## 9.7 動作確認

```sh
# PX4 側で登録状況を確認
pxh> commander status
# → "External Mode 1: nav_state: 23, name: Delivery" のように表示される

pxh> listener register_ext_component_reply
pxh> listener arming_check_reply
pxh> listener setpoint_config
pxh> listener vehicle_status         # nav_state が 23 になっているか
```

`ModeManagement.cpp` にはデバッグ出力があります:

```cpp
PX4_INFO("External Mode %i: nav_state: %i, name: %s", ...);
```

QGroundControl のモード一覧にも自分のモード名が表示されます。

---

## 9.8 制約と注意点

| 項目 | 内容 |
| --- | --- |
| 最大モード数 | **8 個**（`NAVIGATION_STATE_EXTERNAL1..8`） |
| 最大登録数 | アーミングチェック登録も 8 個 |
| API 安定性 | **実験的**。設定値タイプの一部はまだ変更されうる |
| 必要バージョン | PX4 v1.15 以降 |
| 依存 | ROS 2、`px4_msgs`、`px4-ros2-interface-lib`、uXRCE-DDS Agent |
| レイテンシ | ROS 2 経由のため、角速度レベルの制御には不向き |
| コンパニオン必須 | ROS 2 が動く計算機が機体に必要 |

> [!NOTE]
> 公式ドキュメントは `docs/en/ros2/px4_ros2_control_interface.md` および
> `docs/en/ros2/px4_ros2_navigation_interface.md` にあります。
> 実験的機能のため、**PX4 のバージョンとライブラリのバージョンを揃えてください**。

---

## 9.9 Offboard と外部モードの違い（まとめ）

| | Offboard（07 章） | 外部モード（本章） |
| --- | --- | --- |
| PX4 から見た位置づけ | 「よく分からない外部が制御中」 | **「自分のモードの一つ」** |
| GCS の表示 | "Offboard" のみ | **モード名が出る** |
| 同時に持てるモード数 | 1 | **8** |
| アーミングチェック | 参加できない | **参加できる（理由も表示できる）** |
| 必要な推定値の宣言 | 設定値から自動判定 | **明示的に宣言** |
| モードの置き換え | 不可 | **可能（フォールバック付き）** |
| モード完了通知 | なし | **`ModeCompleted`** |
| モード間シーケンス | 自前で実装 | **モードエグゼキュータ** |
| フェイルセーフの一時保留 | 不可 | **`ConfigOverrides`** |
| 応答停止の検知 | 設定値タイムアウトのみ | **アーミングチェックの無応答でも検知** |
| 実装の手間 | 小 | 中（ライブラリが吸収） |
| API 安定性 | 安定 | 実験的 |

---

## 9.10 この章のまとめ

- 外部モードは **ROS 2 のコードを PX4 のフライトモードとして登録**する仕組み
- 枠は `NAVIGATION_STATE_EXTERNAL1..8` の **8 個**
- 登録は `RegisterExtComponentRequest` → `ModeManagement` → `RegisterExtComponentReply`
- **`ArmingCheckReply` で自分の条件を PX4 に伝えられる**（Offboard にはない最大の利点）
- `SetpointConfig` で送る設定値の種類を宣言する
- **PX4 内部モードを置き換えられ、失敗時は元に戻る**
- モードエグゼキュータでモードを跨いだシーケンスを組める
- 実験的 API なので、バージョンを揃えて使うこと

---

[← 前へ: 08. ルート B: PX4 内部に実装](08_route_internal.md) | [目次](README.md) | [次へ: 10. 3 ルートの比較と選定 →](10_route_comparison.md)
