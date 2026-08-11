# 02. uORB と実行モデル

[← 前へ: 01. アーキテクチャ全体像](01_architecture.md) | [目次](README.md) | [次へ: 03. センシングと状態推定 →](03_sensing_estimation.md)

---

この章を理解すれば、**PX4 のどのモジュールでもソースを読めるようになります**。
逆にここが曖昧だと、制御コードを読んでも「この値はどこから来たのか」が追えません。

---

## 2.1 uORB とは何か

uORB (micro Object Request Broker) は PX4 内部の **publish / subscribe メッセージバス**です。

```
   publisher                 uORB                     subscriber
  ┌──────────┐         ┌──────────────┐          ┌──────────────┐
  │  ekf2    │──pub──> │vehicle_attitude│ ──sub──> │mc_att_control│
  └──────────┘         │ (最新値を保持)  │          └──────────────┘
                       └──────────────┘      └──sub──> logger
                                             └──sub──> mavlink
```

特徴:

| 特性 | 意味 |
| --- | --- |
| **最新値保持型** | 各トピックは「最新の 1 個（またはリングバッファ数個）」を保持。キューではない |
| **疎結合** | publisher は誰が読むか知らない。subscriber は誰が書くか知らない |
| **ノンブロッキング** | `publish()` も `copy()` も待たない |
| **スレッドセーフ** | 内部でロックされている |
| **多重インスタンス対応** | 同じ型のトピックを複数持てる（IMU 3 個など） |

実装本体: `platforms/common/uORB/`

| ファイル | 役割 |
| --- | --- |
| `uORB.h` | 公開 API と `ORB_ID()` マクロ |
| `Publication.hpp` | 発行側の C++ ラッパ |
| `Subscription.hpp` | 購読側の C++ ラッパ |
| `SubscriptionCallback.hpp` | **コールバック購読**（PX4 の実行モデルの核） |
| `uORBDeviceNode.cpp` | トピック 1 個分の実体（バッファと通知） |
| `uORBManager.cpp` | トピックの登録・検索 |

---

## 2.2 メッセージ定義（`.msg`）はシステムの API 仕様書

`msg/` 配下の `.msg` ファイルが、そのままトピックの型定義になります。
**ここを読むのが PX4 理解の最短経路です。**

例: `msg/versioned/TrajectorySetpoint.msg`

```
# Trajectory setpoint in NED frame
# Input to PID position controller.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position     # in meters
float32[3] velocity     # in meters/second
float32[3] acceleration # in meters/second^2
float32[3] jerk         # in meters/second^3 (for logging only)

float32 yaw      # euler angle of desired attitude in radians -PI..+PI
float32 yawspeed # angular velocity around NED frame z-axis in radians/second
```

ビルド時にこれが C++ 構造体 `trajectory_setpoint_s` と
ヘッダ `uORB/topics/trajectory_setpoint.h` に自動生成されます。

### 命名規則

| 記法 | 例 | 使う場所 |
| --- | --- | --- |
| ファイル名（UpperCamelCase） | `TrajectorySetpoint.msg` | `msg/` |
| トピック名（snake_case） | `trajectory_setpoint` | `ORB_ID(...)`、`listener` コマンド |
| C++ 構造体 | `trajectory_setpoint_s` | ソースコード |
| ROS 2 型 | `px4_msgs::msg::TrajectorySetpoint` | ROS 2 側 |

### `msg/` と `msg/versioned/` の違い

- `msg/` — PX4 内部専用。予告なく変更されうる
- `msg/versioned/` — **ROS 2 と共有する公開 API**。`MESSAGE_VERSION` を持ち、
  互換性が管理される

> [!IMPORTANT]
> **外部から自律制御する（ルート A / C）なら、`msg/versioned/` のメッセージを使ってください。**
> `msg/` 直下のものは PX4 のバージョンアップで壊れる可能性があります。

### `# TOPICS` 行の意味

一部の `.msg` の末尾にこういう行があります:

```
# TOPICS vehicle_thrust_setpoint
# TOPICS vehicle_thrust_setpoint_virtual_fw vehicle_thrust_setpoint_virtual_mc
```

これは「**同じ構造体を別名のトピックとしても生成せよ**」という指示です。
VTOL では MC 用と FW 用の推力設定値が同時に存在するため、こうした別名が使われます。

---

## 2.3 publish / subscribe の書き方

### 発行する

```cpp
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

// メンバとして宣言（トピック名を ORB_ID で指定）
uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp_pub{ORB_ID(vehicle_attitude_setpoint)};

// 使うとき
vehicle_attitude_setpoint_s sp{};        // ★ {} でゼロ初期化するのが作法
sp.timestamp = hrt_absolute_time();      // ★ タイムスタンプ必須
sp.q_d[0] = 1.f; /* ... */
_attitude_sp_pub.publish(sp);
```

派生クラス:

| クラス | 用途 |
| --- | --- |
| `uORB::Publication<T>` | 通常 |
| `uORB::PublicationMulti<T>` | 多重インスタンス（IMU が複数ある場合など） |
| `uORB::PublicationData<T>` | 構造体をメンバとして保持し `update()` で発行 |

### 購読する（ポーリング型）

```cpp
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

// 使うとき
vehicle_local_position_s pos;
if (_local_pos_sub.update(&pos)) {     // 新しいデータがあれば true でコピー
    // pos を使う
}

// 「新着でなくてもいいから最新値が欲しい」なら copy()
_local_pos_sub.copy(&pos);
```

| メソッド | 挙動 |
| --- | --- |
| `updated()` | 新着があるか真偽で返す（コピーしない） |
| `update(&data)` | 新着があればコピーして `true` |
| `copy(&data)` | 新着かどうかに関係なく最新値をコピー |

### 購読する（コールバック型）★ 最重要

**PX4 の制御モジュールはほぼすべてこの形です。**

```cpp
#include <uORB/SubscriptionCallback.hpp>

class MyController : public ModuleBase, public ModuleParams, public px4::WorkItem
{
    // this を渡すことで「このトピックが publish されたら Run() を呼べ」となる
    uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
};

bool MyController::init()
{
    // ★ ここでコールバックを登録しないと一生走らない
    if (!_angular_velocity_sub.registerCallback()) {
        PX4_ERR("callback registration failed");
        return false;
    }
    return true;
}

void MyController::Run()
{
    // vehicle_angular_velocity が publish されるたびに呼ばれる
}
```

これが **「データが来たら走る」というイベント駆動**の実体です。
タイマーではなく、上流のトピック発行が下流を駆動します。

実例:
- `mc_rate_control` → `vehicle_angular_velocity`（≒ ジャイロレート）で駆動
- `mc_att_control` → `vehicle_attitude`（EKF2 出力）で駆動
- `mc_pos_control` → `vehicle_local_position`（EKF2 出力）で駆動

### レート制限付き購読

```cpp
// パラメータ更新は 1 秒に 1 回見れば十分
uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
```

`flight_mode_manager` は位置更新を 20 ms（50 Hz）に制限しています
（`src/modules/flight_mode_manager/FlightModeManager.cpp` の `init()` 内 `set_interval_us(20_ms)`）。

### 多重インスタンスの購読

```cpp
// IMU が複数ある場合に全部まとめて購読
uORB::SubscriptionMultiArray<estimator_sensor_bias_s> _bias_subs{ORB_ID::estimator_sensor_bias};
```

---

## 2.4 WorkQueue — モジュールはどう実行されるか

PX4 のモジュールには 2 つの実行形態があります。

### (a) WorkQueue アイテム（推奨・大多数）

**複数のモジュールが 1 本のスレッドを共有**します。省メモリで、実機の制約に適しています。

```cpp
class MulticopterRateControl : public ModuleBase, public ModuleParams, public px4::WorkItem
{
    MulticopterRateControl() :
        ModuleParams(nullptr),
        WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)  // ★ どのキューに乗るか
    { }
};
```

WorkQueue の一覧は
`platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp` にあります。

| キュー | 用途 | 優先度 |
| --- | --- | --- |
| `rate_ctrl` | 角速度制御（最も時間にシビア） | 最高クラス |
| `INS0`〜`INS3` | 慣性センサ | 高 |
| `SPI0`〜, `I2C0`〜 | バス別センサドライバ | 高 |
| `nav_and_controllers` | 姿勢・位置制御、FlightTask | 高（センサの次） |
| `hp_default` | 汎用・高優先度 | 中 |
| `lp_default` | 汎用・低優先度 | 低 |
| `ttyS0`〜 | シリアル通信 | 低 |

> [!WARNING]
> **同じ WorkQueue に乗ったモジュールは順番に実行されます。**
> `Run()` の中でブロックしたり重い処理をすると、
> **同じキューの他のモジュール全部が遅延します**。
> 自作モジュールを `rate_ctrl` に乗せるのは、本当に必要な場合だけにしてください。

`px4::WorkItem` と `px4::ScheduledWorkItem` の違い:

| クラス | 駆動方法 |
| --- | --- |
| `WorkItem` | uORB コールバックのみで駆動 |
| `ScheduledWorkItem` | `ScheduleOnInterval()` / `ScheduleDelayed()` で定期実行も可能 |

### (b) 専用タスク（スレッド）

`mavlink`, `logger`, `navigator`, `commander` など、
自前のループを持つ・ブロッキング I/O をする・スタックを多く使うモジュールは独立タスクです。
`ModuleBase::run()` をオーバーライドし、`px4_task_spawn_cmd()` で起動します。

---

## 2.5 モジュールのライフサイクル

すべてのモジュールは `ModuleBase`（`platforms/common/include/px4_platform_common/module.h`）を継承し、
同じインターフェースを持ちます。

```cpp
class MyModule : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
    static Descriptor desc;                          // モジュールの実体を保持

    static int task_spawn(int argc, char *argv[]);   // "start" で呼ばれる
    static int custom_command(int argc, char *argv[]);// 独自サブコマンド
    static int print_usage(const char *reason);      // "help" / 誤用時
    int print_status() override;                     // "status" で呼ばれる

    bool init();                                     // コールバック登録など
private:
    void Run() override;                             // ★ 本体
};

// シェルから呼ばれるエントリポイント
extern "C" __EXPORT int my_module_main(int argc, char *argv[])
{
    return ModuleBase::main(MyModule::desc, argc, argv);
}
```

これにより、**すべてのモジュールが同じコマンド体系**を持ちます:

```sh
my_module start
my_module status
my_module stop
my_module help
```

`Run()` の定型パターン（`src/examples/work_item/WorkItemExample.cpp` がお手本）:

```cpp
void MyModule::Run()
{
    // 1. 停止要求のチェック（必須）
    if (should_exit()) {
        _some_sub.unregisterCallback();
        exit_and_cleanup(desc);
        return;
    }

    perf_begin(_loop_perf);   // 2. 性能計測開始

    // 3. パラメータ更新のチェック
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    // 4. 入力トピックの取得 → 処理 → 出力トピックの発行
    // ...

    perf_end(_loop_perf);     // 5. 性能計測終了
}
```

> [!TIP]
> **`src/examples/work_item/` を丸ごとコピーするのが自作モジュールの正しい出発点です。**
> 手順は [08 章](08_route_internal.md) に書きます。

---

## 2.6 パラメータシステム

PX4 のパラメータは不揮発メモリに保存され、実行時に変更でき、GCS から編集できます。

### 定義（YAML 方式・現在の推奨）

モジュールディレクトリに `*_params.yaml` を置きます。
例: `src/modules/mc_pos_control/multicopter_position_control_gain_params.yaml`

```yaml
parameters:
  - group: Multicopter Position Control
    definitions:
      MPC_XY_P:
        description:
          short: Proportional gain for horizontal position error
        type: float
        min: 0.0
        max: 2.0
        decimal: 2
        default: 0.95
```

### 使用（C++ 側）

```cpp
class MyModule : public ModuleParams
{
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::MPC_XY_P>)  _param_mpc_xy_p,
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart
    )
};

// 読む
float gain = _param_mpc_xy_p.get();

// 書く（通常はやらない。autotune など特殊な用途のみ）
_param_mpc_xy_p.set(1.0f);
_param_mpc_xy_p.commit();
```

`updateParams()` を呼ぶと `DEFINE_PARAMETERS` で宣言した全メンバが最新値に更新されます。
だから 2.5 節の定型パターンで `parameter_update` を購読しているわけです。

### シェルからの操作

```sh
param show MPC_XY_P          # 値を表示
param show MPC_*             # ワイルドカード
param set MPC_XY_P 1.2       # 変更
param save                   # 永続化
param reset MPC_XY_P         # 既定値に戻す
param status                 # 使用状況
```

---

## 2.7 実機と SITL で使う調査コマンド

**これらを使いこなせるかで、デバッグ効率が 10 倍変わります。**
実機なら NuttX シェル（MAVLink コンソール / シリアル）、SITL なら `pxh>` プロンプトで実行します。

### `listener` — トピックの中身を見る

```sh
listener vehicle_local_position          # 1 回表示
listener vehicle_attitude -n 10          # 10 回表示
listener vehicle_angular_velocity -r 5   # 5 Hz に制限して表示
listener sensor_accel -i 1               # インスタンス 1 を表示
```

実装: `src/systemcmds/topic_listener/listener_main.cpp`

> [!TIP]
> **「設定値が届いていない」系のトラブルは、まず `listener` で該当トピックを見ること。**
> 例えば Offboard が効かないなら `listener trajectory_setpoint` と
> `listener offboard_control_mode` を確認します。

### `uorb top` — トピックの発行レートを見る

```sh
uorb top                              # 発行中のトピックをレート順に表示
uorb top -a                           # 購読者がいないものも含めて全部
uorb top vehicle_attitude             # 特定トピックだけ
uorb top -1                           # 1 回だけ表示して終了
uorb status                           # トピック統計
```

実装: `src/systemcmds/uorb/uorb.cpp`

出力例の読み方: `NUM_SUB` が 0 なら誰も聞いていない、`RATE` が想定より低ければ上流が詰まっている、
`LOST` が増えていればバッファが溢れている、と診断できます。

### `work_queue status` — WorkQueue の負荷を見る

```sh
work_queue status
```

どのキューにどのモジュールが乗っていて、どれだけ時間を使っているかが分かります。

### `top` — CPU / メモリ

```sh
top          # タスクごとの CPU 使用率とスタック使用量
top once     # 1 回だけ
```

### `perf` — 性能カウンタ

```sh
perf         # 全モジュールの perf カウンタ（実行時間、実行間隔）
perf reset
```

`perf_begin` / `perf_end` で計測されている区間の統計が見られます。
`mc_rate_control: cycle` の `max` が異常に大きければ、そこがボトルネックです。

### `commander status` / `dmesg`

```sh
commander status    # アーミング状態、フライトモード、フェイルセーフ状態
dmesg               # 起動ログ（起動失敗の原因調査に必須）
```

---

## 2.8 この章のまとめ

- uORB は **最新値保持型の pub/sub バス**。モジュール間の唯一の連結手段
- **`msg/` の `.msg` ファイルがシステムの API 仕様書**。まずここを読む
- 外部連携には `msg/versioned/` を使う（互換性が保証される）
- 制御モジュールは **`SubscriptionCallbackWorkItem` によるイベント駆動**
- モジュールは `ModuleBase` を継承し、共通のコマンド体系を持つ
- パラメータは YAML で定義し、`DEFINE_PARAMETERS` で使う
- **`listener` / `uorb top` / `work_queue status` / `perf` が調査の基本セット**

次章から、実際のデータの流れを上流から追っていきます。

---

[← 前へ: 01. アーキテクチャ全体像](01_architecture.md) | [目次](README.md) | [次へ: 03. センシングと状態推定 →](03_sensing_estimation.md)
