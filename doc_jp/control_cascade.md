# PX4 マルチコプタ制御カスケードの構造とループレート

PX4 のマルチコプタ制御が「どのモジュールが、何に駆動されて、何 Hz で回っているのか」を、ソースコードに即して解説する。
外部制御器 (MPC、学習ポリシーなど) を PX4 に接続する際に必要となる前提知識をまとめたもの。

対象バージョンは本リポジトリの `main` (PX4 v1.16 系)。

---

## 1. 全体像

PX4 のマルチコプタ制御は 4 段のカスケード構成になっている。
外側ほど遅く、内側ほど速い。

```
   目標位置                                                          モータ出力
      │                                                                  ▲
      ▼                                                                  │
 ┌──────────┐   ┌──────────┐   ┌───────────┐   ┌──────────────┐   ┌────────────┐
 │ 位置制御  │──▶│ 姿勢制御  │──▶│ 角速度制御 │──▶│  制御配分     │──▶│ モータ/ESC │
 │mc_pos_   │sp │mc_att_   │sp │mc_rate_   │sp │control_      │   │            │
 │control   │   │control   │   │control    │   │allocator     │   │            │
 └──────────┘   └──────────┘   └───────────┘   └──────────────┘   └────────────┘
   ~100 Hz        200 Hz          400 Hz            400 Hz
      ▲               ▲                ▲
      │               │                │
 vehicle_local_  vehicle_        vehicle_angular_
 position        attitude        velocity
 (EKF2)          (EKF2)          (ジャイロ)
```

各段の間は uORB トピックでつながっている。

| 段 | 出力トピック | 内容 |
|---|---|---|
| 位置制御 | `vehicle_attitude_setpoint` | 目標姿勢 (クォータニオン) + 推力ベクトル |
| 姿勢制御 | `vehicle_rates_setpoint` | 目標角速度 (rad/s) + 推力ベクトル |
| 角速度制御 | `vehicle_torque_setpoint` / `vehicle_thrust_setpoint` | 正規化トルク / 推力 |
| 制御配分 | `actuator_motors` | 各モータの正規化出力 |

公式の図は `docs/en/flight_stack/controller_diagrams.md` にある。

---

## 2. 「ループレート」を決めているのは何か

**PX4 の制御モジュールは固定周期タイマーで回っていない。**
購読している uORB トピックが更新されたときにコールバックで起こされる。

`src/modules/mc_rate_control/MulticopterRateControl.cpp`:

```cpp
bool MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
	return true;
}
```

つまり:

> **モジュールのループレート = 駆動トピックの publish レート**

これが PX4 のレートを理解する上での最重要事項である。
「角速度制御を速くしたい」と思ったら、`mc_rate_control` を触るのではなく `vehicle_angular_velocity` の publish レートを上げる。

各モジュールの駆動トピックは以下のとおり。

| モジュール | 駆動トピック | 登録箇所 |
|---|---|---|
| `mc_pos_control` | `vehicle_local_position` | `MulticopterPositionControl.cpp:64` |
| `mc_att_control` | `vehicle_attitude` | `mc_att_control_main.cpp:83` |
| `mc_rate_control` | `vehicle_angular_velocity` | `MulticopterRateControl.cpp:69` |

---

## 3. 各段の実レートと決定パラメータ

デフォルト設定での実際のレートは次のとおり。

| 段 | 実レート (default) | 決定パラメータ | パラメータの default |
|---|---|---|---|
| ジャイロ生データ | **~8 kHz** | センサドライバ | — |
| **角速度制御 (内側)** | **400 Hz** | `IMU_GYRO_RATEMAX` | **400** |
| 姿勢制御 | **200 Hz** | `IMU_INTEG_RATE` | **200** |
| 位置制御 | **~100 Hz** | `EKF2_PREDICT_US` | **10000 µs** |

### `IMU_GYRO_RATEMAX` — 内側ループレートそのもの

`src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.yaml`:

```yaml
IMU_GYRO_RATEMAX:
  description:
    short: Gyro control data maximum publication rate (inner loop rate)
    long: |-
      The maximum rate the gyro control data (vehicle_angular_velocity) will be
      allowed to publish at. This is the loop rate for the rate controller and outputs.

      Note: sensor data is always read and filtered at the full raw rate (eg commonly 8 kHz) regardless of this setting.
  type: enum
  values:
    100: 100 Hz
    250: 250 Hz
    400: 400 Hz
    800: 800 Hz
    1000: 1000 Hz
    2000: 2000 Hz
  default: 400
```

パラメータ名の説明文に "inner loop rate"、"This is the loop rate for the rate controller and outputs" と明記されている。

ここで区別すべき2つのレートがある。

- **サンプリング/フィルタリングレート** — 常に生データレート (~8 kHz)。この設定に影響されない
- **publish レート** — `IMU_GYRO_RATEMAX` で間引かれる。これが制御ループレートになる

間引きの実装は `VehicleAngularVelocity.cpp:134-156` の `UpdateSampleRate()` にある。
`set_required_updates(samples)` で uORB コールバックを N サンプルごとにしか発火させないため、
レート制御器はちょうど `IMU_GYRO_RATEMAX` で起こされる。

> **注意**: よく「PX4 の角速度ループは 1 kHz」と言われるが、**デフォルトは 400 Hz** である。
> 1 kHz にしたければ `IMU_GYRO_RATEMAX` を明示的に設定する (再起動が必要)。

---

## 4. `mc_rate_control` の読み解き

外部制御器を接続するときに一番重要になるモジュールなので、詳しく見る。

### 4.1 ループ本体はジャイロ更新でゲートされている

`MulticopterRateControl.cpp:125-134`:

```cpp
	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;
```

`dt` は 0.125 ms 〜 20 ms にクランプされる。
これは 50 Hz 〜 8 kHz の範囲を想定しているということ。

なお `mc_rate_control` には **backup schedule がない**。
固定翼版の `FixedwingRateControl.cpp:217` には 20 ms の watchdog があるが、マルチコプタ版にはない。
ジャイロの publish が止まればモジュールも止まる。

### 4.2 角速度 setpoint はラッチされる ← 最重要

`MulticopterRateControl.cpp:181-186`:

```cpp
	} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
		_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
		_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
		_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
		_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
	}
```

`update()` は**新しいサンプルが届いたときだけ true を返す**。
そして `_rates_setpoint` と `_thrust_setpoint` は**クラスメンバ**である。

`MulticopterRateControl.hpp:125-130` — コメントで明示されている:

```cpp
	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};
```

そして PID 本体は、setpoint が更新されたかどうかに関係なく毎回走る (`:189-220`):

```cpp
	// run the rate controller
	if (_vehicle_control_mode.flag_control_rates_enabled) {
		...
		// run rate controller
		Vector3f torque_setpoint =
			_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
```

**結論: 角速度 setpoint はゼロ次ホールド (ZOH) される。**

setpoint が 400 Hz で来ようが 10 Hz で来ようが、レート PID 自体は毎ジャイロサンプル、
**最新の実測角速度に対して** 400 Hz で回り続ける。
外側の setpoint レートは内側ループの帯域に影響しない。

これはカスケード制御の設計意図そのものである。

### 4.3 NaN 軸の扱い

上のコードの `PX4_ISFINITE(...) ? ... : rates(n)` に注目。
setpoint の軸が NaN の場合、**その軸には現在の実測角速度が代入される**。
結果としてその軸の誤差はゼロになり、制御されない。

エラーではなく「その軸は指定しない」という意味の正常な入力として扱われる。

### 4.4 陳腐化チェックが存在しない ← 安全上の注意

**`vehicle_rates_setpoint` の timestamp を検査しているコードは `src/` 全体に存在しない。**

つまり publisher が死んでも、レート制御器は**最後に受け取った角速度と推力を無限に保持し続ける**。
唯一の保護は commander 側の offboard failsafe だが、これは別トピックを見ている (第 6 節)。

対比として、RC 入力には同じハザードへの警告が明記されている。
`src/modules/commander/commander_params.yaml` の `COM_RC_LOSS_T`:

> The time in seconds without a new setpoint from RC or Joystick, after which the connection is considered lost.
> **This must be kept short as the vehicle will use the last supplied setpoint until the timeout triggers.**

角速度 setpoint には同等の記述も仕組みもない。
外部制御器を接続する側が watchdog を実装する必要がある。

---

## 5. `rate_control.cpp` の PID 構造

`src/lib/rate_control/rate_control.cpp:71-86`:

```cpp
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}
```

各項の意味:

| 項 | 入力 | パラメータ |
|---|---|---|
| P | 角速度誤差 `rate_sp - rate` | `MC_ROLLRATE_P` 他 |
| I | 誤差の積分 | `MC_ROLLRATE_I` 他 |
| **D** | **実測角加速度 `angular_accel`** | `MC_ROLLRATE_D` 他 |
| FF | setpoint そのもの | `MC_ROLLRATE_FF` 他 (default 0) |

### D 項が「誤差の微分」ではない点が重要

一般的な PID では D 項は誤差の微分 $d(e)/dt$ を使う。
PX4 は違い、**実測角加速度** (`vehicle_angular_velocity.xyz_derivative`) を使っている。

この帰結:

> **setpoint が階段状に飛んでも微分キック (derivative kick) が起きない。**

誤差微分型だと setpoint のステップが $d(e)/dt$ に巨大なインパルスを生み、トルク指令が跳ねる。
測定値微分型ではそれが原理的に起こらない。

**したがって PX4 のレート制御器は、構造的に低レート・階段状の setpoint に耐性がある。**
外部制御器から粗い setpoint を投げ込む場合に効いてくる性質である。

なお `angular_accel` は生ジャイロを微分してから LPF をかけたもので、
カットオフは `IMU_DGYRO_CUTOFF` (default 20 Hz)。
生成箇所は `VehicleAngularVelocity.cpp:777-789`。

---

## 6. `vehicle_control_mode` によるループの有効化・無効化

どの段を PX4 が担当し、どの段を外部に明け渡すかは `vehicle_control_mode` のフラグで決まる。

offboard モードでのマッピングは `src/modules/commander/ModeUtil/control_mode.cpp:111-143`:

```cpp
	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		vehicle_control_mode.flag_control_offboard_enabled = true;

		if (offboard_control_mode.position) {
			getControlMode(SetpointType::Trajectory, vehicle_control_mode);
		} else if (offboard_control_mode.velocity) {
			...
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

**優先順位付きの `else if` チェーン**である点に注意。
`position` と `body_rate` を両方 true にすると `position` が勝つ。

そして `src/modules/commander/ModeUtil/setpoint_types.cpp:85-94`:

```cpp
	case SetpointType::Rates:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		break;

	case SetpointType::Attitude:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;
		break;
```

`SetpointType::Rates` では `flag_control_attitude_enabled` が立たない。
→ **`mc_att_control` は完全に非稼働になり、`vehicle_rates_setpoint` の publisher は外部制御器だけになる。**

---

## 7. 外部制御器をどこに差し込めるか

カスケードの各段が注入点になる。
`offboard_control_mode` のどのフィールドを true にするかで決まる。

| `offboard_control_mode` | 書き込むトピック | PX4 が担当し続ける範囲 | 外部側が担当する範囲 |
|---|---|---|---|
| `position` / `velocity` | `TrajectorySetpoint` | 位置・姿勢・角速度・配分 | 軌道生成のみ |
| `attitude` | `VehicleAttitudeSetpoint` | **姿勢 (200Hz)**・角速度・配分 | 位置制御 |
| `body_rate` | `VehicleRatesSetpoint` | **角速度 (400Hz)**・配分 | 位置制御 + 姿勢制御 |
| `thrust_and_torque` | `VehicleThrustSetpoint` / `VehicleTorqueSetpoint` | 配分のみ | 位置 + 姿勢 + 角速度 |
| `direct_actuator` | `ActuatorMotors` | なし | すべて |

**下に行くほど外部側の責務が増え、要求される更新レートも上がる。**

`docs/en/concept/flight_modes.md` には、外部モードが適さないケースとして以下が挙げられている:

> Modes that require **low-level access, strict timing, and/or high update rate requirements**.
> For example, a multicopter mode that implements direct motor control.

リポジトリ内の実装例:

- `src/modules/mc_raptor/` — 強化学習ポリシーが `actuator_motors` を直接出力 (カスケード全体をバイパス)
- `src/modules/mc_nn_control/` — ニューラルネット制御器。uORB フローの途中に割り込むパターンの実例

---

## 8. 実レートの確認方法

実機/SITL で実際のレートを見るには `uorb top` を使う。

```
nsh> uorb top
```

`vehicle_angular_velocity`、`vehicle_attitude`、`vehicle_local_position`、`vehicle_rates_setpoint` の
publish レートが表示される。設定値と実測値がずれていないかの確認に使う。

ログ解析では各メッセージの `timestamp_sample` の差分を見る。
`timestamp` (publish 時刻) ではなく `timestamp_sample` (センサ取得時刻) を使うのが正しい。

---

## 9. 用語の注意: `MPC_*` は Model Predictive Control ではない

`docs/en/contribute/notation.md` に明記されている:

> | MPC or MCPC | MultiCopter Position Controller. MPC is also used for Model Predictive Control. |

PX4 のパラメータ接頭辞 `MPC_*` (`MPC_XY_VEL_P_ACC`、`MPC_THR_HOVER`、`MPC_XY_CRUISE` など) は
すべて**マルチコプタ位置制御器**のパラメータである。
モデル予測制御とはまったく関係がない。

---

## 参考

- `docs/en/flight_stack/controller_diagrams.md` — 制御ブロック図 (公式)
- `docs/en/concept/architecture.md` — uORB とモジュール構成
- `docs/en/flight_modes/offboard.md` — offboard モードの仕様
- `docs/en/concept/flight_modes.md` — 外部モードの適用範囲
