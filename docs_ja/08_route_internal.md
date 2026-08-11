# 08. ルート B: PX4 内部に実装する

[← 前へ: 07. ルート A: Offboard 制御](07_route_offboard.md) | [目次](README.md) | [次へ: 09. ルート C: px4_ros2 外部モード →](09_route_external_mode.md)

---

**PX4 のファームウェア自体に自分のコードを組み込む**ルートです。

- 制御ループと同じレートで動く（最高 1 kHz）
- レイテンシがほぼゼロ
- 通信断の影響を受けない
- **代償**: 開発サイクルが遅い、リソース制約が厳しい、C++ 縛り

**独自の制御則を試したい人には、これしか選択肢がありません。**

---

## 8.1 3 つのアプローチ

| | 追加するもの | 難易度 | 適した用途 |
| --- | --- | --- | --- |
| **B-1** | 新規モジュール | 中 | 独自の推定・制御・監視ロジック |
| **B-2** | 新規 FlightTask | 低 | 新しい飛行モード（軌道生成） |
| **B-3** | 既存モジュールの改造 | 高 | 制御則そのものの変更 |

---

## 8.2 B-1: 新規モジュールを追加する

### 出発点は `src/examples/work_item/`

**これを丸ごとコピーするのが正しいやり方です。** 一から書く必要はありません。

```sh
cp -r src/examples/work_item src/modules/my_controller
cd src/modules/my_controller
mv WorkItemExample.cpp MyController.cpp
mv WorkItemExample.hpp MyController.hpp
```

> `src/templates/template_module/` はスレッド型モジュールの雛形です。
> WorkQueue 型（推奨）なら `src/examples/work_item/` を使ってください。

### 必要なファイル

```
src/modules/my_controller/
├── CMakeLists.txt          ← ビルド定義
├── Kconfig                 ← ビルド設定への登録
├── MyController.cpp        ← 実装
├── MyController.hpp        ← ヘッダ
└── my_controller_params.yaml  ← パラメータ定義（任意）
```

#### `CMakeLists.txt`

```cmake
px4_add_module(
	MODULE modules__my_controller
	MAIN my_controller
	SRCS
		MyController.cpp
		MyController.hpp
	DEPENDS
		px4_work_queue
		mathlib
	MODULE_CONFIG
		my_controller_params.yaml
	)
```

| 引数 | 意味 |
| --- | --- |
| `MODULE` | 一意なモジュール名。`ディレクトリ__名前` の慣習 |
| `MAIN` | **シェルから叩くコマンド名**。`<MAIN>_main()` が呼ばれる |
| `SRCS` | ソースファイル |
| `DEPENDS` | 依存ライブラリ（`px4_work_queue`, `mathlib`, `matrix`, `geo` など） |
| `MODULE_CONFIG` | パラメータ定義 YAML |
| `STACK_MAIN` | スタックサイズ（スレッド型のとき） |
| `EXTERNAL` | ツリー外モジュール |

定義は `cmake/px4_add_module.cmake` にあります。

#### `Kconfig`

```
menuconfig MODULES_MY_CONTROLLER
	bool "my_controller"
	default n
	---help---
		Enable support for my_controller
```

`src/modules/Kconfig` が `rsource "*/Kconfig"` で全サブディレクトリを取り込むので、
**ファイルを置くだけで自動的に認識されます。**

命名規則: `MODULES_` + ディレクトリ名の大文字（`src/modules/` の下の場合）。
`src/examples/` の下なら `EXAMPLES_`、`src/drivers/` の下なら `DRIVERS_` です。

#### ボード定義への追加 ★ 忘れやすい

```sh
# boards/px4/sitl/default.px4board に追記
CONFIG_MODULES_MY_CONTROLLER=y
```

または対話的に:

```sh
make px4_sitl_default boardconfig
```

> [!WARNING]
> **これを忘れるとビルドに含まれず、`my_controller: command not found` になります。**
> 実機で使うときは、そのボードの `.px4board` にも追加が必要です。

#### 起動スクリプトへの追加

自動起動させたい場合は、機体タイプの起動スクリプトに追記します:

```sh
# ROMFS/px4fmu_common/init.d/rc.mc_apps
if param compare -s MY_CTRL_EN 1
then
	my_controller start
fi
```

**パラメータで条件起動するのが PX4 流**です。

### 実装のポイント

```cpp
// MyController.hpp
class MyController : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	static Descriptor desc;

	MyController();
	~MyController() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	// ★ 駆動トピック: これが publish されるたびに Run() が呼ばれる
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};

	// 参照するトピック
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// 出力トピック
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MY_CTRL_GAIN>) _param_my_ctrl_gain
	)
};
```

```cpp
// MyController.cpp
MyController::MyController() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)  // ★ キュー選択
{
}

bool MyController::init()
{
	if (!_local_pos_sub.registerCallback()) {   // ★ これを忘れると走らない
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}

void MyController::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		// ★ 制御モードを確認してから動く
		vehicle_control_mode_s control_mode;
		_control_mode_sub.copy(&control_mode);

		if (control_mode.flag_control_offboard_enabled) {
			trajectory_setpoint_s sp{};
			sp.position     = {NAN, NAN, -5.0f};
			sp.velocity     = {NAN, NAN, NAN};
			sp.acceleration = {NAN, NAN, NAN};
			sp.yaw          = NAN;
			sp.yawspeed     = NAN;
			sp.timestamp    = hrt_absolute_time();
			_trajectory_setpoint_pub.publish(sp);
		}
	}

	perf_end(_loop_perf);
}
```

### WorkQueue の選び方

| キュー | 使いどころ |
| --- | --- |
| `nav_and_controllers` | 位置・姿勢制御、軌道生成（**大半はここ**） |
| `rate_ctrl` | 角速度制御レベルの処理のみ |
| `hp_default` | 制御ではないが応答性が必要 |
| `lp_default` | 監視・ログなど遅くてよいもの |

> [!WARNING]
> **`Run()` を軽く保ってください。** 同じキューの他モジュールがブロックされます。
> 重い処理（動的メモリ確保、ファイル I/O、`printf`）は避けます。
> 目安は `perf` の `max` が周期の 1/3 以下。

### パラメータの定義

`my_controller_params.yaml`:

```yaml
module_name: My Controller
parameters:
  - group: My Controller
    definitions:
      MY_CTRL_EN:
        description:
          short: Enable my controller
        type: int32
        min: 0
        max: 1
        default: 0
        reboot_required: true
      MY_CTRL_GAIN:
        description:
          short: Control gain
          long: |
            Proportional gain of my custom controller.
        type: float
        min: 0.0
        max: 10.0
        decimal: 2
        increment: 0.1
        unit: 'm/s'
        default: 1.0
```

**パラメータ名は最大 16 文字**です。既存の接頭辞（`MPC_`, `MC_`, `COM_` など）とは
衝突しないものを選んでください。

### ビルドと確認

```sh
make px4_sitl_default
make px4_sitl gz_x500

pxh> my_controller start
pxh> my_controller status
pxh> work_queue status         # どのキューに乗ったか
pxh> perf                      # 実行時間
pxh> listener trajectory_setpoint
```

---

## 8.3 B-2: 新規 FlightTask を追加する

**新しい飛行モード（軌道生成）を作るなら、これが最も筋の良い方法です。**
PX4 の位置制御・姿勢制御・安全機構をそのまま使えます。

### 手順

#### ① ディレクトリとファイルを作る

```
src/modules/flight_mode_manager/tasks/MyTask/
├── CMakeLists.txt
├── FlightTaskMyTask.cpp
└── FlightTaskMyTask.hpp
```

既存の単純なタスク（`tasks/Descend/`, `tasks/Failsafe/`）をコピーするのが早道です。

#### ② `CMakeLists.txt`

```cmake
px4_add_library(FlightTaskMyTask
	FlightTaskMyTask.cpp
)
target_link_libraries(FlightTaskMyTask PUBLIC FlightTask)
target_include_directories(FlightTaskMyTask PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
```

#### ③ 実装

```cpp
// FlightTaskMyTask.hpp
#pragma once
#include "FlightTask.hpp"

class FlightTaskMyTask : public FlightTask
{
public:
	FlightTaskMyTask() = default;
	virtual ~FlightTaskMyTask() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool update() override;
};
```

```cpp
// FlightTaskMyTask.cpp
#include "FlightTaskMyTask.hpp"

bool FlightTaskMyTask::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	// ここで初期化。_position（現在位置）などが使える
	return ret;
}

bool FlightTaskMyTask::update()
{
	// ★ メンバに設定値を書けば、それが trajectory_setpoint になる
	//   制御しない軸は NAN
	_position_setpoint     = matrix::Vector3f(NAN, NAN, -5.0f);
	_velocity_setpoint     = matrix::Vector3f(1.0f, 0.0f, NAN);
	_acceleration_setpoint = matrix::Vector3f(NAN, NAN, NAN);
	_yaw_setpoint          = NAN;
	_yawspeed_setpoint     = NAN;

	return true;   // false を返すとタスク失敗 → フォールバック
}
```

利用できる基底クラスのメンバ（現在の状態）:

| メンバ | 内容 |
| --- | --- |
| `_position` | 現在位置（ローカル NED） |
| `_velocity` | 現在速度 |
| `_yaw` | 現在方位 |
| `_dist_to_bottom` | 地面までの距離 |
| `_deltatime` | 前回からの経過時間 [s] |
| `_time_stamp_current` | 現在時刻 |

#### ④ 登録

`src/modules/flight_mode_manager/CMakeLists.txt`:

```cmake
list(APPEND flight_tasks_all
	Auto
	Descend
	Failsafe
	ManualAcceleration
	...
	MyTask          # ← 追加するだけ
)
```

**`FlightTaskIndex` 列挙型もファクトリも自動生成されます**
（`generate_flight_tasks.py` + `Templates/FlightTasks_generated.*.em`）。

#### ⑤ タスクを起動する条件を書く

`src/modules/flight_mode_manager/FlightModeManager.cpp` の
`start_flight_task()` に条件を追加します:

```cpp
if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_MY_MODE) {
	found_some_task = true;
	if (switchTask(FlightTaskIndex::MyTask) != FlightTaskError::NoError) {
		matching_task_running = false;
		task_failure = true;
	}
}
```

> [!NOTE]
> **新しい `nav_state` を足すのは大仕事です。**
> `VehicleStatus.msg` への追加、commander のモード管理、MAVLink のモード対応、
> QGroundControl 側の表示など、広範囲に影響します。
>
> **新しいモードが欲しいだけなら、[外部モード（09 章）](09_route_external_mode.md)の
> `NAVIGATION_STATE_EXTERNAL1..8` 枠を使うほうが圧倒的に簡単です。**
>
> FlightTask 追加が有効なのは、既存モード（例: Orbit や Follow Target）の
> 挙動を差し替える／改良する場合です。

---

## 8.4 B-3: 既存の制御則を差し替える

**独自の制御アルゴリズム（MPC、L1、適応制御、学習ベース）を試す場合**です。

### アプローチ 1: 制御ライブラリを差し替える

制御則の本体はモジュールから分離されています:

| 段 | 制御則ライブラリ | モジュール |
| --- | --- | --- |
| 位置 | `src/modules/mc_pos_control/PositionControl/` | `mc_pos_control` |
| 姿勢 | `src/modules/mc_att_control/AttitudeControl/` | `mc_att_control` |
| 角速度 | `src/lib/rate_control/` | `mc_rate_control` |

**`PositionControl` クラスと同じインターフェースを持つクラスを作り、
モジュール側の型を差し替える**のが最小の変更です。

```cpp
// MulticopterPositionControl.hpp
// PositionControl _control;      ← これを
MyPositionControl _control;       // ← こう変える
```

必要なインターフェース（`PositionControl.hpp` 参照）:

```cpp
void setState(const PositionControlStates &states);
void setInputSetpoint(const trajectory_setpoint_s &setpoint);
bool update(const float dt);
void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;
void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;
```

### アプローチ 2: 並行モジュールで上書きする

**既存モジュールを止めて、自分のモジュールを同じトピックの publisher にする**方法です。

```sh
# ROMFS/px4fmu_common/init.d/rc.mc_apps を編集
# mc_pos_control start          ← コメントアウト
my_pos_control start            # 自分のモジュール
```

自分のモジュールが `vehicle_attitude_setpoint` を publish すれば、
`mc_att_control` 以下はそのまま動きます。

> [!TIP]
> **こちらのほうがマージが楽で、既存コードとの差分も小さくなります。**
> 実験段階ではこちらを推奨します。

### 実例: 既存の代替制御モジュール

PX4 本体に、まさにこの方式の実装例があります:

| モジュール | 内容 |
| --- | --- |
| `src/modules/mc_nn_control/` | ニューラルネットワーク制御（`MC_NN_EN=1` で起動） |
| `src/modules/mc_raptor/` | 学習ベース制御（`MC_RAPTOR_ENABLE=1` で起動） |

`src/modules/mc_raptor/README.md` には SITL での動かし方まで書かれています。
**独自制御則を組み込む際の最良の参考実装です。**

起動スクリプト（`rc.mc_apps`）での条件起動:

```sh
if param compare -s MC_NN_EN 1
then
	mc_nn_control start
fi

if param compare -s MC_RAPTOR_ENABLE 1
then
	mc_raptor start
fi
```

### 使える計算ライブラリ

| ライブラリ | 内容 |
| --- | --- |
| `src/lib/matrix/` | 行列・ベクトル・クォータニオン（ヘッダオンリー、動的確保なし） |
| `src/lib/mathlib/` | フィルタ、`constrain`、補間 |
| `src/lib/motion_planning/` | 躍度制限つき軌道生成 |
| `src/lib/pid/`, `src/lib/pid_design/` | PID とゲイン設計 |
| `src/lib/system_identification/` | システム同定 |
| `src/lib/tensorflow_lite_micro/` | TFLite Micro（学習モデルの推論） |
| `src/lib/rl_tools/` | 強化学習ツール |

> [!WARNING]
> **フライトコードでは動的メモリ確保（`new` / `malloc`）を避けてください。**
> リアルタイム性を損ない、断片化の原因になります。
> 初期化時（コンストラクタ）に確保し、`Run()` の中では確保しないのが原則です。

---

## 8.5 開発サイクルを速くする

内部実装の最大の弱点は開発サイクルの遅さです。緩和策:

### SITL で回す

```sh
make px4_sitl gz_x500       # ビルドして起動まで一発
```

差分ビルドなら 10〜30 秒程度です。

### 単体テストを書く

制御則を独立したクラスにしておけば、実機なしでテストできます。
PX4 には Google Test が組み込まれています。

既存の例:
- `src/modules/mc_pos_control/PositionControl/PositionControlTest.cpp`
- `src/modules/mc_att_control/AttitudeControl/AttitudeControlTest.cpp`
- `src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverseTest.cpp`

```sh
make tests                          # 全テスト
make tests TESTFILTER=PositionControl   # 絞り込み
```

**制御則の開発は、まず単体テストで数値を確認するのが最速です。**

### ログで検証する

```sh
# SITL 実行後、build/px4_sitl_default/rootfs/log/ に .ulg が出る
```

[Flight Review](https://review.px4.io/) や PlotJuggler で可視化します（[11 章](11_sitl_workflow.md)）。

### コーディング規約

```sh
make format          # 自動整形（コミット前に必須）
make check_format    # CI と同じチェック
```

PX4 のスタイルは 4 幅タブ、K&R 系です。`.clang-format` に定義されています。

---

## 8.6 この章のまとめ

- **`src/examples/work_item/` をコピーするのが新規モジュールの正しい出発点**
- 必要なのは `CMakeLists.txt` + `Kconfig` + **`.px4board` への追加**（忘れやすい）
- `WorkItem` + `SubscriptionCallbackWorkItem` でイベント駆動にする
- **`Run()` は軽く保つ**。同じ WorkQueue の他モジュールに影響する
- 新しい飛行モードなら **FlightTask 追加**が筋が良い（`flight_tasks_all` に足すだけ）
  - ただし **新しい `nav_state` を足すのは大仕事**。外部モード枠を検討する
- 独自制御則は「**制御ライブラリを差し替える**」か
  「**並行モジュールで既存を置き換える**」
- `mc_nn_control` / `mc_raptor` が実装済みの参考例
- 動的メモリ確保を避け、単体テストで検証し、`make format` を通す

---

[← 前へ: 07. ルート A: Offboard 制御](07_route_offboard.md) | [目次](README.md) | [次へ: 09. ルート C: px4_ros2 外部モード →](09_route_external_mode.md)
