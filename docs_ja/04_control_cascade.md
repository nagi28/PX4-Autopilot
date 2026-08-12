# 04. 制御カスケード ★本書の中核

[← 前へ: 03. センシングと状態推定](03_sensing_estimation.md) | [目次](README.md) | [次へ: 05. モード管理と安全機構 →](05_modes_safety.md)

---

**この章が本書の核心です。** マルチコプターの制御は 4 段のカスケードで構成されており、
自律制御を実装するとは「**このどこかに自分の設定値を注入すること**」に他なりません。

---

## 制御チェーン全体図

```
┌─────────────────────────────────────────────────────────────────────┐
│  設定値の生成元（どれか 1 つが trajectory_setpoint を発行）              │
│   ・flight_mode_manager（FlightTask） ← navigator / RC スティック       │
│   ・外部プログラム（Offboard / 外部モード）                              │
└─────────────────────────────────────────────────────────────────────┘
                            │
                  trajectory_setpoint
              (位置[3] / 速度[3] / 加速度[3] / yaw / yawspeed)
                            │
  ┌─────────────────────────▼─────────────────────────────────────┐
  │ ① mc_pos_control                                                │
  │    src/modules/mc_pos_control/                                  │
  │    入力状態: vehicle_local_position                              │
  │    位置 P →速度PID→加速度→推力ベクトル→姿勢                     │
  └─────────────────────────┬─────────────────────────────────────┘
                            │
                vehicle_attitude_setpoint
                  (q_d[4] クォータニオン, thrust_body[3])
                            │
  ┌─────────────────────────▼─────────────────────────────────────┐
  │ ② mc_att_control                                                │
  │    src/modules/mc_att_control/                                  │
  │    入力状態: vehicle_attitude                                    │
  │    クォータニオン誤差の P 制御 + 2次参照モデル                     │
  └─────────────────────────┬─────────────────────────────────────┘
                            │
                 vehicle_rates_setpoint
                (roll/pitch/yaw [rad/s], thrust_body[3])
                            │
  ┌─────────────────────────▼─────────────────────────────────────┐
  │ ③ mc_rate_control                                               │
  │    src/modules/mc_rate_control/                                 │
  │    入力状態: vehicle_angular_velocity                            │
  │    角速度 PID（+ 配分器からの飽和フィードバック）                  │
  └─────────────────────────┬─────────────────────────────────────┘
                            │
        vehicle_thrust_setpoint  /  vehicle_torque_setpoint
              (正規化 [-1,1] の推力ベクトルとトルクベクトル)
                            │
  ┌─────────────────────────▼─────────────────────────────────────┐
  │ ④ control_allocator                                             │
  │    src/modules/control_allocator/                               │
  │    効果行列の擬似逆行列で各モータ／サーボへ配分                    │
  └─────────────────────────┬─────────────────────────────────────┘
                            │
              actuator_motors / actuator_servos
                            │
                            ▼
              pwm_out / dshot / uavcan ドライバ → ESC → モータ
```

### 各段の駆動レートと責務

| 段 | モジュール | 駆動トピック | 典型レート | 責務 |
| --- | --- | --- | --- | --- |
| ① | `mc_pos_control` | `vehicle_local_position` | 50 Hz | どこへ行くか → どう傾くか |
| ② | `mc_att_control` | `vehicle_attitude` | 250 Hz | どう傾くか → どう回るか |
| ③ | `mc_rate_control` | `vehicle_angular_velocity` | 250〜1000 Hz | どう回るか → どんな力／トルクか |
| ④ | `control_allocator` | `vehicle_torque_setpoint` | ③ と同じ | 力／トルク → 各モータ出力 |

**上の段ほど遅く、下の段ほど速い。** これがカスケード制御の基本です。
外側のループは内側のループが十分速く追従することを前提に設計されています。

> [!IMPORTANT]
> **「駆動トピック」の数と、制御が使う物理量の数は違います。**
> PX4 は関連する物理量を 1 つのメッセージにまとめて配るので、
> トピックが 3 つでも中身はもっと多いです。
>
> | トピック | 同梱されている物理量 |
> | --- | --- |
> | `vehicle_local_position` | **位置**（`x/y/z`）＋ **速度**（`vx/vy/vz`）＋ 加速度（`ax/ay/az`）＋ 方位 ＋ 妥当性フラグ |
> | `vehicle_attitude` | 姿勢クォータニオン（`q`） |
> | `vehicle_angular_velocity` | **角速度**（`xyz`）＋ **角加速度**（`xyz_derivative`） |
>
> つまり位置制御は位置だけでなく **速度もフィードバックしています**。
> 実際、位置制御の主役は速度 PID であり、位置 P 制御は
> 「速度設定値を作る」ためだけに存在します。
> 速度信号の詳しい扱いは [`_vel_dot` はどこから来るのか](#_vel_dot-はどこから来るのか) を参照してください。

---

## NaN イディオム

**PX4 の制御を理解するうえで最も重要な約束事です。**

設定値メッセージのフィールドに `NaN` を入れると、
**「その自由度は制御しない（この段では何も指令しない）」** という意味になります。

```
position    = {NaN, NaN, -5.0}   → 水平位置は制御しない、高度 5 m は保持
velocity    = {2.0, 0.0, NaN}    → 水平は速度 2 m/s 北、垂直は速度指令なし
acceleration= {NaN, NaN, NaN}    → 加速度フィードフォワードなし
yaw         = NaN                → 方位は制御しない（現在方位を保持）
```

この組み合わせで「速度制御モード」「位置制御モード」「高度のみ制御」などを表現します。
**モードを表す別のフラグは要りません。設定値の形そのものがモードです。**

### 妥当性チェックの実装

`src/modules/mc_pos_control/PositionControl/PositionControl.cpp` の `_inputValid()`:

```cpp
bool PositionControl::_inputValid()
{
    bool valid = true;

    // 各軸 x,y,z は「位置・速度・加速度のいずれか」が有限値でなければならない
    for (int i = 0; i <= 2; i++) {
        valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
    }

    // x と y は必ずペアで指定する（片方だけ NaN は禁止）
    valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
    valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
    valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

    // 制御する状態については、推定値も有効でなければならない
    for (int i = 0; i <= 2; i++) {
        if (PX4_ISFINITE(_pos_sp(i))) { valid = valid && PX4_ISFINITE(_pos(i)); }
        if (PX4_ISFINITE(_vel_sp(i))) { valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i)); }
    }
    return valid;
}
```

> [!WARNING]
> **ルール:**
> 1. 各軸に最低 1 つは有限の設定値が必要（全部 NaN は不正）
> 2. **x と y は必ずペア**。「北方向だけ位置指定、東方向は速度指定」は不可
> 3. 制御対象の推定値が無効なら制御されない
>
> `ROS 2` から送るときは、**構造体をゼロ初期化すると 0 が入ってしまう**点に注意。
> 「制御しない」なら明示的に `NaN` を代入してください。

### 加算の仕組み

位置・速度・加速度は排他ではなく **加算**されます
（`ControlMath::addIfNotNanVector3f`、`src/modules/mc_pos_control/PositionControl/ControlMath.cpp`）。

```
速度設定値 = 位置誤差 × P ゲイン  +  フィードフォワード速度設定値
加速度設定値 = 速度 PID 出力  +  フィードフォワード加速度設定値
```

つまり「位置 + 速度 + 加速度」を全部指定すると、
**位置がフィードバック、速度と加速度がフィードフォワード**として働きます。
これが滑らかな軌道追従の作り方です。

---

## ① mc_pos_control — 位置・速度制御

実装: `src/modules/mc_pos_control/`

| ファイル | 役割 |
| --- | --- |
| `MulticopterPositionControl.cpp` | uORB 入出力、モード処理、離陸ランプ |
| `PositionControl/PositionControl.cpp` | **制御則の本体** |
| `PositionControl/ControlMath.cpp` | 推力ベクトル → 姿勢の変換、NaN 処理 |
| `Takeoff/Takeoff.cpp` | スムーズ離陸のステートマシン |
| `GotoControl/GotoControl.cpp` | `goto_setpoint` の平滑化 |

### 制御則の 3 段構成

`PositionControl::update()` が呼ぶ順序:

#### (1) 位置 P 制御 — `_positionControl()`

```cpp
Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);

// 水平速度の制限（位置由来の成分を優先し、FF 成分を削る）
_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(),
                                        (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
```

- ゲイン: `MPC_XY_P`（水平）, `MPC_Z_P`（垂直）
- 制限: `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN`
- **速度制限時は位置誤差由来の成分が優先される**（フィードフォワードを先に削る）

#### (2) 速度 PID 制御 — `_velocityControl(dt)`

```cpp
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
```

- ゲイン: `MPC_XY_VEL_P_ACC` / `_I_ACC` / `_D_ACC`、`MPC_Z_VEL_P_ACC` / `_I_ACC` / `_D_ACC`
- **D 項は速度誤差の微分ではなく、加速度 `_vel_dot` を使う**
  （設定値が変化した瞬間の微分キックを避けるため）
- 出力の単位は **加速度 [m/s²]**（PX4 では加速度が制御の共通通貨）

##### `_vel_dot` はどこから来るのか

ここは誤解しやすい箇所です。`_vel_dot` は
**`vehicle_local_position` の `ax/ay/az` ではありません**。
`mc_pos_control` はそのフィールドを一切読んでおらず、
**自分でフィルタ済み速度を微分して作っています**。

`MulticopterPositionControl::set_vehicle_states()`
（`src/modules/mc_pos_control/MulticopterPositionControl.cpp`）:

```cpp
const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();

// 速度: ノッチフィルタ → ローパスフィルタ
states.velocity.xy() = _vel_xy_lp_filter.update(_vel_xy_notch_filter.apply(velocity_xy));

// 加速度: 上のフィルタ済み速度を差分し、さらにローパス
states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((_vel_xy_lp_filter.getState() - vel_xy_prev) / dt_s);
```

`PositionControl::setState()` がこれを受け取ります
（`_vel = states.velocity`、`_vel_dot = states.acceleration`）。

したがって位置制御が使う信号の流れはこうなります:

```
vehicle_local_position
  ├─ x / y / z        ──────────────────────────────→ _pos      （位置 P 制御）
  ├─ vx / vy / vz  ─┬─ ノッチ → LPF ────────────────→ _vel      （速度 P・I 項）
  │                 └─ 差分 / dt → LPF ─────────────→ _vel_dot  （速度 D 項）
  └─ ax / ay / az     ── mc_pos_control は使わない
```

フィルタのパラメータ:

| パラメータ | 既定値 | 役割 |
| --- | --- | --- |
| `MPC_VEL_NF_FRQ` | 0 Hz（無効） | 速度のノッチフィルタ中心周波数 |
| `MPC_VEL_NF_BW` | 5 Hz | 同ノッチの帯域幅 |
| `MPC_VEL_LP` | 0 Hz（無効） | 速度のローパス遮断周波数 |
| `MPC_VELD_LP` | **5 Hz（有効）** | **速度微分のローパス遮断周波数（D 項のノイズ対策）** |

> [!IMPORTANT]
> **要点は「微分を避ける」ではなく「微分を 1 段に留めてフィルタする」です。**
>
> - **位置 → 速度** の微分は **しません**。EKF2 が速度を独立した状態量として
>   観測融合で推定しています（[03 章](03_sensing_estimation.md#状態ベクトル)）。
>   位置を数値微分するとノイズが増幅されるためです。
> - **速度 → 加速度** の微分は **します**。ただしノッチとローパスを前後に挟みます。
>
> 速度推定が無効になったとき（`v_xy_valid` が false）は
> **フィルタもリセット**されます。そうしないと速度が復帰した瞬間に
> 差分が巨大になり、D 項が加速度スパイクを生むためです。

#### (3) 加速度 → 推力ベクトル → 姿勢 — `_accelerationControl()`

```cpp
float z_specific_force = -CONSTANTS_ONE_G;
if (!_decouple_horizontal_and_vertical_acceleration) {
    z_specific_force += _acc_sp(2);       // 垂直加速度も考慮（MPC_ACC_DECOUPLE）
}
// 望む加速度と重力の合成方向 = 機体 Z 軸の向き
Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);   // 最大傾斜角で制限

// ホバリング推力を基準に加速度→推力へ換算
const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
_thr_sp = body_z * collective_thrust;
```

これがマルチコプター制御の**核心のアイデア**です:

> マルチコプターは推力を機体 Z 軸方向にしか出せない。
> したがって「望む加速度ベクトル + 重力」の方向に機体 Z 軸を向ければよい。
> **加速度指令は、そのまま姿勢指令に変換できる。**

- 傾斜制限: `MPC_TILTMAX_AIR`（飛行中）, `MPC_TILTMAX_LND`（離着陸時）
- ホバリング推力: `MPC_THR_HOVER`（`mc_hover_thrust_estimator` が実測から更新）

最後に `ControlMath::thrustToAttitude()` が推力ベクトルと yaw 設定値から
クォータニオン `q_d` を作り、`vehicle_attitude_setpoint` として発行します。

### 垂直優先の推力配分

推力には限りがあるので、水平と垂直で奪い合いになります。PX4 は **垂直を優先**します
（高度を失うほうが危険なため）。ただし完全に水平を潰さないよう
`MPC_THR_XY_MARG` のマージンを確保します。

```cpp
// 水平にマージン分だけ残して、垂直に使える最大推力を決める
const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);
_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

// 垂直を決めた後の残りを水平に配る
const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
```

- 推力制限: `MPC_THR_MIN`, `MPC_THR_MAX`, `MPC_THR_XY_MARG`

### アンチワインドアップ（積分の暴走防止）

2 種類が併用されています。

**垂直方向 — 条件付き積分停止:**

```cpp
if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
    vel_error(2) = 0.f;    // 飽和方向へさらに積むのをやめる
}
```

**水平方向 — トラッキング型アンチワインドアップ:**

```cpp
// 実際に出せた加速度と、要求した加速度の差を使って誤差を割り引く
const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
    const float arw_gain = 2.f / _gain_vel_p(0);
    vel_error.xy() = Vector2f(vel_error) - arw_gain * (_acc_sp.xy() - acc_sp_xy_produced);
}
```

> [!TIP]
> **なぜ重要か:** 強風下で機体が押し戻され続けると、積分項が際限なく溜まります。
> 風が止んだ瞬間に溜まった積分が一気に効いて機体が飛び出します（ワインドアップ）。
> 独自の制御則を書くなら、**必ず同等の対策を入れてください**。

### ホバリング推力の自動推定

`mc_hover_thrust_estimator` が `hover_thrust_estimate` トピックを発行し、
`mc_pos_control` が `updateHoverThrust()` で受け取ります。

重要なのは、**ホバリング推力が変わっても積分項が飛ばないように補正している**点です
（バッテリ消費で機体が軽くなると `MPC_THR_HOVER` 相当値が下がります）:

```cpp
void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
    const float previous_hover_thrust = _hover_thrust;
    setHoverThrust(hover_thrust_new);
    if (PX4_ISFINITE(_acc_sp(2))) {
        _vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
                       + CONSTANTS_ONE_G - _acc_sp(2);
    }
}
```

### EKF リセットへの追従

`MulticopterPositionControl::adjustSetpointForEKFResets()` が
`vehicle_local_position` のリセットカウンタを監視し、
ジャンプ分だけ設定値をずらします（[03 章](03_sensing_estimation.md#リセットカウンタ)参照）。

### 設定値が来ないときのフェイルセーフ

位置制御が有効なのに設定値が届かない場合、
`generateFailsafeSetpoint()` が安全な設定値（緩やかな降下）を生成します。
**外部から設定値を送るプログラムが止まっても、いきなり墜落はしません。**

---

## ② mc_att_control — 姿勢制御

実装: `src/modules/mc_att_control/`

| ファイル | 役割 |
| --- | --- |
| `mc_att_control_main.cpp` | uORB 入出力、パラメータ、手動モード処理 |
| `AttitudeControl/AttitudeControl.cpp` | **制御則の本体** |

### クォータニオン誤差による P 制御

`AttitudeControl::update()` の骨子:

```cpp
// 姿勢誤差クォータニオン qe = q^-1 * qd
const Quatf qe = qinv(q) * qd;

// 回転軸 × sin(角度/2) を誤差ベクトルとする（正準化で最短回転を選ぶ）
const Vector3f eq = 2.f * qe.canonical().imag();

// 角速度設定値 = 誤差 × P ゲイン
Vector3f rate_setpoint = eq.emult(_proportional_gain);
```

- ゲイン: `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P`
- 制限: `MC_ROLLRATE_MAX`, `MC_PITCHRATE_MAX`, `MC_YAWRATE_MAX`

> **なぜクォータニオンか:** オイラー角だとジンバルロック（ピッチ 90°）で破綻します。
> クォータニオンなら全姿勢で連続的に誤差が定義できます。アクロバット飛行にも耐えます。

### ロール・ピッチ優先（yaw weight）

**マルチコプター制御の重要な設計判断です。**

推力方向を決めるのはロールとピッチであり、
yaw はどちらを向くかを決めるだけです。だから **傾き（tilt）の修正を優先し、
yaw の修正は後回し**にします。

```cpp
// 現在と目標の機体 Z 軸から「傾きだけを合わせる姿勢」qd_red を作る
const Vector3f e_z   = q.dcm_z();
const Vector3f e_z_d = qd.dcm_z();
Quatf qd_red(e_z, e_z_d);
qd_red *= q;

// 完全な目標姿勢 qd から yaw 成分だけを取り出す
Quatf qd_dyaw = qinv(qd_red) * qd;

// yaw 成分を _yaw_w (= MC_YAW_WEIGHT) 倍に縮めて合成
qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0, 0, sinf(_yaw_w * asinf(qd_dyaw(3))));
```

- 重み: `MC_YAW_WEIGHT`（0〜1、小さいほど yaw を後回しにする）

> [!TIP]
> **機体が yaw を合わせようとして位置を崩す場合、`MC_YAW_WEIGHT` を下げてください。**
> ヨー方向のトルクは反トルクで作るため、そもそも他軸より弱いという物理的制約もあります。

### 2 次参照モデル

このバージョンの PX4 は、目標姿勢をそのまま追わず、
**臨界減衰の 2 次系フィルタを通した「参照姿勢」**を追います。

`AttitudeControl::propagateReferenceModel()`:

```cpp
// A = [0 -1; kq -2*omega_n] の行列指数を厳密（ZOH）に離散化
// 固有値が -omega_n の重根なので閉じた形になる
const float w_dt  = _omega_n * dt;
const float emt   = expf(-w_dt);
const float a     = (1.f + w_dt) * emt;
const float b     = dt * emt;
const float gamma = _kq * dt * emt;
const float delta = (1.f - w_dt) * emt;

const Vector3f delta_phi = (1.f - a) * e + b * _omega_correction + omega_command * dt;
_omega_correction = gamma * e + delta * _omega_correction;
_q_ref = _q_ref * Quatf(AxisAnglef(delta_phi));
```

**効果:**
1. 姿勢設定値がステップ状に変化しても、機体への指令が滑らかになる
2. 参照モデルが予測する角速度を **フィードフォワード**として使えるので、追従が速くなる

- 帯域: `MC_REF_W_N`（自然角周波数 [rad/s]）
- FF ゲイン: `MC_REF_FF`、上限: `MC_REF_FF_MAX`

出力は `vehicle_rates_setpoint` として発行されます。

---

## ③ mc_rate_control — 角速度制御

実装: `src/modules/mc_rate_control/` + 制御則ライブラリ `src/lib/rate_control/`

**最も内側かつ最も速いループ**です。ここが不安定だと機体は必ず落ちます。

### PID 制御則

`RateControl::update()`（`src/lib/rate_control/rate_control.cpp`）:

```cpp
Vector3f rate_error = rate_sp - rate;
Vector3f torque = _gain_p.emult(rate_error)          // P
                + _rate_int                          // I
                - _gain_d.emult(angular_accel)       // D（角加速度を直接使う）
                + _gain_ff.emult(rate_sp);           // FF
```

- **D 項は角速度誤差の微分ではなく、`vehicle_angular_velocity.xyz_derivative`（角加速度）を使う**
  → 設定値変化による微分キック（derivative kick）が起きない
- 出力は **正規化トルク [-1, 1]**

> [!NOTE]
> **位置ループとの対比 — D 項の作り方が 2 段で違います。**
>
> | | D 項の入力 | 誰が微分するか |
> | --- | --- | --- |
> | 位置ループ（`mc_pos_control`） | 速度の微分 | **制御器が自分で**（`set_vehicle_states()`） |
> | 角速度ループ（`mc_rate_control`） | `vehicle_angular_velocity.xyz_derivative` | **`sensors` 側が済ませている** |
>
> 角速度側は上流の `VehicleAngularVelocity`
> （`src/modules/sensors/vehicle_angular_velocity/`）が
> ノッチ・ローパスをかけた上で角加速度まで算出し、トピックに載せて配ります。
> ジャイロは 1 kHz 級で届くので、フィルタ処理を上流に一箇所へ集めたほうが
> 合理的だからです。位置ループ側は 50 Hz なので制御器内で完結させています。

### ゲインの構成

PX4 のレート PID は「並列形」を「理想形」に変換して使います:

```cpp
const Vector3f rate_k = Vector3f(MC_ROLLRATE_K, MC_PITCHRATE_K, MC_YAWRATE_K);
_rate_control.setPidGains(
    rate_k.emult(Vector3f(MC_ROLLRATE_P, MC_PITCHRATE_P, MC_YAWRATE_P)),
    rate_k.emult(Vector3f(MC_ROLLRATE_I, MC_PITCHRATE_I, MC_YAWRATE_I)),
    rate_k.emult(Vector3f(MC_ROLLRATE_D, MC_PITCHRATE_D, MC_YAWRATE_D)));
```

`MC_*RATE_K` は全体ゲイン。**チューニング時はまず K を上下させると P/I/D のバランスを保ったまま
全体の強さを変えられます。**

| パラメータ | 役割 |
| --- | --- |
| `MC_ROLLRATE_K` / `MC_PITCHRATE_K` / `MC_YAWRATE_K` | 全体ゲイン |
| `MC_ROLLRATE_P` / `_I` / `_D` | ロール PID |
| `MC_PITCHRATE_P` / `_I` / `_D` | ピッチ PID |
| `MC_YAWRATE_P` / `_I` / `_D` | ヨー PID |
| `MC_RR_INT_LIM` / `MC_PR_INT_LIM` / `MC_YR_INT_LIM` | 積分上限 |
| `MC_YAW_TQ_CUTOFF` | ヨートルク出力のローパス周波数 |

### 配分器からの飽和フィードバック ★

**PX4 の巧妙な点です。** モータが飽和して要求トルクを出せなかった場合、
`control_allocator` が `control_allocator_status` で「配分できなかったトルク」を報告し、
レート制御器はその方向への積分を止めます。

```cpp
if (_control_allocator_status_sub.update(&control_allocator_status)) {
    if (!control_allocator_status.torque_setpoint_achieved) {
        for (size_t i = 0; i < 3; i++) {
            if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
                saturation_positive(i) = true;
            } else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
                saturation_negative(i) = true;
            }
        }
    }
    _rate_control.setSaturationStatus(saturation_positive, saturation_negative);
}
```

`RateControl::updateIntegral()` 側:

```cpp
if (_control_allocator_saturation_positive(i)) { rate_error(i) = math::min(rate_error(i), 0.f); }
if (_control_allocator_saturation_negative(i)) { rate_error(i) = math::max(rate_error(i), 0.f); }

// さらに、誤差が大きいほど I ゲインを下げる（フリップ後の跳ね返り防止）
float i_factor = rate_error(i) / math::radians(400.f);
i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;
```

### バッテリ電圧補償

`MC_BAT_SCALE_EN=1` にすると、バッテリ電圧の低下に応じて出力をスケールします。
これがないと、飛行中盤以降にゲインが実質的に下がって応答が鈍くなります。

```cpp
vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
```

### 出力

`vehicle_thrust_setpoint`（推力ベクトル）と `vehicle_torque_setpoint`（トルクベクトル）。
どちらも正規化された `[-1, 1]` の値で、**まだ物理単位ではありません**。

> [!NOTE]
> **Acro モード（手動角速度制御）もこのモジュールが処理します。**
> `flag_control_manual_enabled && !flag_control_attitude_enabled` のとき、
> スティック入力から直接角速度設定値を作ります（`MC_ACRO_*` パラメータ）。

---

## ④ control_allocator — 推力配分

実装: `src/modules/control_allocator/` + `src/lib/control_allocation/`

「機体全体に対する推力とトルク」を「各モータ・サーボの出力」へ変換します。
**ここが機体形状（クアッド / ヘキサ / VTOL / ヘリ）の違いを吸収します。**

### 効果行列（Effectiveness Matrix）

各アクチュエータが機体にどんな力・トルクを与えるかを表す行列 `B` を作ります。

```
[ トルク_x ]            [ モータ1出力 ]
[ トルク_y ]            [ モータ2出力 ]
[ トルク_z ]  =  B   ×  [ モータ3出力 ]
[ 推力_x  ]            [ モータ4出力 ]
[ 推力_y  ]            [    ...      ]
[ 推力_z  ]
```

機体タイプごとの実装が `src/modules/control_allocator/VehicleActuatorEffectiveness/` にあります:

| クラス | 機体 |
| --- | --- |
| `ActuatorEffectivenessMultirotor` | マルチコプター |
| `ActuatorEffectivenessRotors` | ロータ群の共通処理（位置・軸方向・回転方向から B を構築） |
| `ActuatorEffectivenessFixedWing` | 固定翼 |
| `ActuatorEffectivenessStandardVTOL` / `TiltrotorVTOL` / `TailsitterVTOL` | VTOL |
| `ActuatorEffectivenessHelicopter` | ヘリコプター |
| `ActuatorEffectivenessMCTilt` / `Tilts` | チルトロータ |
| `ActuatorEffectivenessCustom` | **カスタム機体** |

ロータの配置はパラメータで定義されます（QGroundControl の Actuators 画面で設定）:

| パラメータ | 意味 |
| --- | --- |
| `CA_AIRFRAME` | 機体タイプ |
| `CA_ROTOR_COUNT` | ロータ数 |
| `CA_ROTOR{n}_PX/PY/PZ` | ロータ n の位置 [m] |
| `CA_ROTOR{n}_AX/AY/AZ` | ロータ n の推力軸方向 |
| `CA_ROTOR{n}_KM` | トルク係数（正負で回転方向） |

### 配分アルゴリズム

`CA_METHOD` で選択します。

| 値 | クラス | 特徴 |
| --- | --- | --- |
| 0 | `ControlAllocationPseudoInverse` | 効果行列の擬似逆行列 `B⁺` を掛けるだけ。単純・高速 |
| 1 | `ControlAllocationSequentialDesaturation` | **既定**。飽和時に優先度順（ヨー → 推力 → ロール/ピッチ）で削る |

逐次デサチュレーションが重要なのは、**飽和したときに何を諦めるかを制御できる**からです。
姿勢（ロール・ピッチ）を守り、ヨーを最初に諦めるのが安全です。

### 出力と後処理

`ControlAllocator::Run()` の流れ:

```
setControlSetpoint(c)        推力・トルク設定値をセット
  → allocate()               擬似逆行列 or 逐次デサチュレーション
  → allocateAuxilaryControls()  フラップ・スポイラー等
  → updateSetpoint()         機体固有の後処理
  → applySlewRateLimit(dt)   変化率制限
  → clipActuatorSetpoint()   [min, max] にクリップ
  → publish                  actuator_motors / actuator_servos
```

さらに:
- 停止すべきモータには `NaN` を書き込む（`applyNanToActuators`）→ ドライバ側で出力を止める
- 実際に配分できた量との差分を `control_allocator_status` で報告 → ③ のアンチワインドアップへ
- `CA_FAILURE_MODE` でモータ故障時の挙動を設定

### そして出力ドライバへ

`actuator_motors` は `src/lib/mixer_module/` を介して各出力ドライバへ渡ります:

| ドライバ | 用途 |
| --- | --- |
| `pwm_out` | 標準 PWM / OneShot |
| `dshot` | DShot デジタル ESC プロトコル |
| `uavcan` / `cyphal` | CAN 経由の ESC |
| `pwm_out_sim` | SITL / HITL |

正規化値 `[-1, 1]` から実際のパルス幅への変換は、出力チャンネルごとのパラメータ
（`PWM_MAIN_MIN1`〜, `PWM_MAIN_MAX1`〜, `PWM_MAIN_DIS1`〜, `PWM_MAIN_FAIL1`〜）で決まります。
これらは `src/drivers/pwm_out/module.yaml` の `param_prefix` と `standard_params` から
ビルド時に自動生成されます（生成器は `Tools/module_config/generate_params.py`）。

---

## どの階層に注入できるか

**自律制御を実装するうえで最も重要な設計判断です。**

```
   外部プログラムからの注入点            必要な自前実装        PX4 が担保するもの
 ┌───────────────────────────────┬──────────────┬────────────────────┐
 │ goto_setpoint                 │ 経路計画のみ    │ 平滑化・全制御・安全      │
 │  （位置 + 方位、平滑化あり）      │              │                     │
 ├───────────────────────────────┼──────────────┼────────────────────┤
 │ trajectory_setpoint       ★推奨│ 軌道生成       │ 位置PID以下すべて       │
 │  （位置/速度/加速度/yaw）        │              │                     │
 ├───────────────────────────────┼──────────────┼────────────────────┤
 │ vehicle_attitude_setpoint     │ 位置・速度制御   │ 姿勢制御以下           │
 │  （クォータニオン + 推力）        │              │                     │
 ├───────────────────────────────┼──────────────┼────────────────────┤
 │ vehicle_rates_setpoint        │ 位置〜姿勢制御   │ 角速度PIDと推力配分     │
 │  （角速度 + 推力）               │              │                     │
 ├───────────────────────────────┼──────────────┼────────────────────┤
 │ vehicle_thrust/torque_setpoint│ 角速度制御まで   │ 推力配分のみ           │
 ├───────────────────────────────┼──────────────┼────────────────────┤
 │ actuator_motors               │ 全部           │ 何も（出力ドライバのみ）  │
 │  （各モータ直接指令）             │              │                     │
 └───────────────────────────────┴──────────────┴────────────────────┘
   ↑ 上ほど安全・簡単             ↓ 下ほど自由・高速だが危険
```

### 選び方の指針

| やりたいこと | 注入点 |
| --- | --- |
| ウェイポイント巡航、探索飛行、追従 | `goto_setpoint` または `trajectory_setpoint` |
| 軌道最適化（MPC など）の結果を流す | `trajectory_setpoint`（加速度 FF 付き） |
| 独自の位置制御則を試したい | `vehicle_attitude_setpoint` |
| アクロバット制御、独自の姿勢制御 | `vehicle_rates_setpoint` |
| 強化学習ポリシーで直接モータを叩く | `actuator_motors`（※実験用途のみ） |

> [!WARNING]
> **`vehicle_rates_setpoint` より下に注入する場合、レイテンシとジッタが致命的になります。**
> 外部（ROS 2 / MAVLink）から 1 kHz 級のループを回すのは現実的ではありません。
> その場合は [08 章](08_route_internal.md) の「PX4 内部に実装」を選んでください。

### 各段の ON/OFF を決めるのは `vehicle_control_mode`

各制御モジュールは `vehicle_control_mode`（`msg/VehicleControlMode.msg`）のフラグを見て、
自分が動くべきかを判断します。

| フラグ | 効果 |
| --- | --- |
| `flag_multicopter_position_control_enabled` | `mc_pos_control` が動く |
| `flag_control_attitude_enabled` | `mc_att_control` が動く |
| `flag_control_rates_enabled` | `mc_rate_control` が動く |
| `flag_control_allocation_enabled` | `control_allocator` が動く |
| `flag_control_position_enabled` / `_velocity_` / `_altitude_` / `_climb_rate_` | どの状態を制御するか |
| `flag_control_offboard_enabled` | Offboard 制御中 |
| `flag_control_manual_enabled` | 手動入力が混ざる |

このフラグを立てるのは **`commander`** です（[05 章](05_modes_safety.md)）。

> [!IMPORTANT]
> **「設定値を送っているのに機体が動かない」場合、まず `listener vehicle_control_mode` を確認してください。**
> 該当フラグが false なら、そもそもその制御段が走っていません。
> フラグはフライトモードによって決まります。

---

## この章のまとめ

- 制御は **位置 → 姿勢 → 角速度 → 推力配分** の 4 段カスケード
- 上流ほど遅く安全、下流ほど速く自由
- **`NaN` = 「その自由度は制御しない」**。設定値の形がそのままモードを表す
- 位置・速度・加速度は加算される（位置=FB、速度/加速度=FF）
- 加速度指令 → 推力ベクトル → 姿勢、という変換がマルチコプター制御の核心
- 垂直推力を優先し、水平にはマージンを残す
- アンチワインドアップは**位置制御・角速度制御の両方**に入っている
- 配分器 → レート制御器への **飽和フィードバック**がループを閉じている
- 外部から注入する階層は `trajectory_setpoint` が最もバランスが良い
- **`vehicle_control_mode` のフラグが各段の動作可否を決める**

---

## 主要パラメータ早見表（マルチコプター制御）

| 段 | パラメータ | 意味 |
| --- | --- | --- |
| 位置 | `MPC_XY_P`, `MPC_Z_P` | 位置 P ゲイン |
| 位置 | `MPC_XY_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 水平速度 PID |
| 位置 | `MPC_Z_VEL_P_ACC` / `_I_ACC` / `_D_ACC` | 垂直速度 PID |
| 位置 | `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP/DN` | 速度上限 |
| 位置 | `MPC_ACC_HOR`, `MPC_ACC_UP_MAX`, `MPC_ACC_DOWN_MAX` | 加速度上限 |
| 位置 | `MPC_TILTMAX_AIR` | 最大傾斜角 |
| 位置 | `MPC_THR_HOVER`, `MPC_THR_MIN`, `MPC_THR_MAX`, `MPC_THR_XY_MARG` | 推力 |
| 位置 | `MPC_JERK_MAX`, `MPC_JERK_AUTO` | 躍度上限（平滑化） |
| 姿勢 | `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | 姿勢 P ゲイン |
| 姿勢 | `MC_YAW_WEIGHT` | ヨーの優先度（下げるとロール/ピッチ優先） |
| 姿勢 | `MC_REF_W_N`, `MC_REF_FF`, `MC_REF_FF_MAX` | 参照モデル |
| 姿勢 | `MC_ROLLRATE_MAX`, `MC_PITCHRATE_MAX`, `MC_YAWRATE_MAX` | 角速度上限 |
| 角速度 | `MC_ROLLRATE_K` / `_P` / `_I` / `_D` | ロール角速度 PID |
| 角速度 | `MC_PITCHRATE_K` / `_P` / `_I` / `_D` | ピッチ角速度 PID |
| 角速度 | `MC_YAWRATE_K` / `_P` / `_I` / `_D` | ヨー角速度 PID |
| 角速度 | `MC_BAT_SCALE_EN` | バッテリ電圧補償 |
| 角速度 | `MC_AIRMODE` | 低スロットル時も姿勢制御を維持するか |
| 配分 | `CA_AIRFRAME`, `CA_ROTOR_COUNT`, `CA_ROTOR{n}_*` | 機体形状定義 |
| 配分 | `CA_METHOD` | 配分アルゴリズム |
| センサ | `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF` | ジャイロ／角加速度フィルタ |

---

[← 前へ: 03. センシングと状態推定](03_sensing_estimation.md) | [目次](README.md) | [次へ: 05. モード管理と安全機構 →](05_modes_safety.md)
