# PAMPC を PX4 に接続する実装手順

コンパニオンコンピュータ上の PAMPC ([uzh-rpg/rpg_mpc](https://github.com/uzh-rpg/rpg_mpc)) の出力
$\mathbf{u} = [c, \mathbf{\Omega}_B]$ を、PX4 の offboard モードに流し込むための手順書。

レート整合性に関する理論的な議論は `doc_pampc/rate_mismatch.md` を参照。
PX4 側の制御構造は `doc_jp/control_cascade.md` を参照。

---

## 1. 経路の選択 — uXRCE-DDS か MAVLink か

角速度 + 推力を送る経路は 2 つある。

| | uXRCE-DDS (ROS 2) | MAVLink |
|---|---|---|
| メッセージ | `VehicleRatesSetpoint` | `SET_ATTITUDE_TARGET` |
| レイテンシ | 低い (共有メモリ / UDP 直結) | 高め (パース + シリアル) |
| 高レート | 得意 | シリアル帯域に律速される |
| nav_state ゲート | **なし** (直接 uORB に publish される) | あり (OFFBOARD 時のみ publish) |
| 推奨 | **こちら** | レガシー構成のみ |

**PAMPC のような高レート外部制御器では uXRCE-DDS を使う。**

### uXRCE-DDS 側にゲートがないことの意味

MAVLink 経路 (`src/modules/mavlink/mavlink_receiver.cpp:1876-1881`) には保護がある:

```cpp
			// Publish rate setpoint only once in OFFBOARD
			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				setpoint.timestamp = hrt_absolute_time();
				_rates_sp_pub.publish(setpoint);
			}
```

一方 uXRCE-DDS 側 (`src/modules/uxrce_dds_client/dds_topics.h.em:235-243`) は素通しである:

```cpp
if (ucdr_deserialize_@(sub['simple_base_type'])(*ub, data, time_offset_us)) {
	pubs->@(sub['topic_simple'])_pub.publish(data);
}
```

→ **ROS 2 ノードが publish すれば、機体のモードに関係なく uORB に書き込まれる。**

`docs/en/flight_modes/offboard.md` の警告どおり、外部制御器側が責任を持つ:

> PX4 has no means of filtering and distinguishing ROS 2 messages from internal messages, in any mode.
> In order to interwork safely, the external controller must: Publish PX4 setpoint messages **ONLY** in Offboard mode

**offboard モード以外では絶対に publish しないこと。** PX4 内部の publisher と衝突する。

---

## 2. 最小構成

### 2.1 送るべき 2 つのトピック

```
ROS 2 ノード                              PX4
    │
    ├── /fmu/in/offboard_control_mode ──▶ OffboardControlMode  (heartbeat)
    │       body_rate = true                 ≥ 2 Hz 必須
    │
    └── /fmu/in/vehicle_rates_setpoint ──▶ VehicleRatesSetpoint (実指令)
            roll/pitch/yaw + thrust_body     MPC のレート
```

`OffboardControlMode` (`msg/OffboardControlMode.msg`):

```
uint64 timestamp
bool position
bool velocity
bool acceleration
bool attitude
bool body_rate        # ← これだけ true
bool thrust_and_torque
bool direct_actuator
```

`VehicleRatesSetpoint` (`msg/versioned/VehicleRatesSetpoint.msg`):

```
uint64 timestamp
# body angular rates in FRD frame
float32 roll          # [rad/s]
float32 pitch         # [rad/s]
float32 yaw           # [rad/s]
float32[3] thrust_body  # Normalized thrust command in body NED frame [-1,1]
bool reset_integral
```

> **注意**: `reset_integral` はマルチコプタでは**無視される**。
> 使っているのは `fw_rate_control` / `fw_att_control` だけ。

### 2.2 publish の順序

1. offboard に入る**前から** `OffboardControlMode` を流し始める (1 秒以上)
2. `VehicleRatesSetpoint` も同時に流し始める
3. モード切替 + arm
4. 飛行中は両方を出し続ける

`docs/en/flight_modes/offboard.md:22`:

> PX4 requires that the external controller provides a continuous 2Hz "proof of life" signal […]
> The stream should be active before switching to Offboard mode

---

## 3. 座標系・符号・単位の変換

PAMPC (ROS / ENU / FLU 系) と PX4 (NED / FRD 系) は座標系が違う。

### 3.1 角速度

PAMPC の $\mathbf{\Omega}_B$ は機体座標系の角速度 [rad/s]。
PX4 の `roll` / `pitch` / `yaw` は **FRD** (Front-Right-Down) 機体座標系の角速度 [rad/s]。

RPG スタックは **FLU** (Front-Left-Up) なので、y と z の符号を反転する:

```cpp
msg.roll  =  omega_B.x();
msg.pitch = -omega_B.y();
msg.yaw   = -omega_B.z();
```

`px4_ros_com` の `frame_transforms` ユーティリティ (`baselink_to_aircraft_body_frame()`) を使ってもよい。

### 3.2 推力

**ここが最も間違えやすい。**

PAMPC の $c$ は**質量正規化 collective thrust** [m/s²] である。
論文 §III-B: $c = (f_1+f_2+f_3+f_4)/m$。ホバリング時は $c = g = 9.81$。

PX4 の `thrust_body[2]` は**正規化スロットル** [-1, 1] で、**下向き (body-Z 正方向) が負**。
メッセージのコメント:

> For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.

線形近似での変換:

$$
\texttt{thrust\_body[2]} = -\frac{c}{g} \cdot T_{hover}
$$

ここで $T_{hover}$ はホバリング時の正規化スロットル。
`MPC_THR_HOVER` (default **0.5**) がその初期値。

```cpp
constexpr float kGravity = 9.80665f;
const float hover_thrust = 0.5f;   // MPC_THR_HOVER と合わせる

msg.thrust_body[0] = 0.f;
msg.thrust_body[1] = 0.f;
msg.thrust_body[2] = -math::constrain(c / kGravity * hover_thrust, 0.f, 1.f);
```

**精度を上げるには:**

1. **`MPC_THR_HOVER` を機体ごとに較正する。** Position モードでホバリングさせ、
   `hover_thrust_estimate` トピック (`msg/HoverThrustEstimate.msg`) の推定値を読む
2. **スロットルと推力の非線形性を考慮する。** `THR_MDL_FAC` (`src/lib/mixer_module/motor_params.yaml`)
   がモータ指令と静推力の関係をモデル化している。これを 0 のままにすると
   スロットル指令はモータ回転数に比例し、推力はその 2 乗に近くなる
3. **バッテリ電圧補償。** `MC_BAT_SCALE_EN` を有効にすると PX4 側で補正される

較正がずれていると、MPC が意図した加速度と実際の加速度が一致せず、高度制御が発散する。
**最初は Position モードでホバリング推力を実測してから offboard に入ること。**

---

## 4. パラメータ設定

| パラメータ | default | 推奨値 | 理由 |
|---|---|---|---|
| `IMU_GYRO_RATEMAX` | 400 | **400〜1000** | 内側ループレート。アグレッシブ機動なら 800/1000。要再起動 |
| `COM_OF_LOSS_T` | 1.0 | **0.2〜0.5** | offboard 喪失の検出時間。短いほど安全 |
| `COM_OBL_RC_ACT` | 0 (Position) | **0 または 4 (Land)** | offboard 喪失時の動作 |
| `COM_DISARM_LAND` | — | **-1** (無効) | offboard 離陸時に着陸検出が誤作動して空中 disarm する |
| `MPC_THR_HOVER` | 0.5 | **実測値** | 推力変換の基準 |
| `THR_MDL_FAC` | 0 | 実測に応じて | スロットル→推力の非線形性 |
| `IMU_DGYRO_CUTOFF` | 20 | 20〜30 | D 項の LPF。高レート機動時は上げる |
| `MC_ROLLRATE_P` 他 | 機体依存 | 要調整 | レート PID ゲイン |

### `COM_DISARM_LAND` について

`src/modules/mc_raptor/README.md` に実例がある:

```bash
param set COM_DISARM_LAND -1 # When taking off in offboard the landing detector can cause mid-air disarms
```

offboard で離陸するとき、着陸検出器が「着陸している」と誤判定して自動 disarm することがある。
外部制御器での離陸では無効化しておく。

### `COM_OF_LOSS_T` の実際の粒度

health check が走るのは 100 ms ごと (`src/modules/commander/Commander.cpp:2050`):

```cpp
	if ((now >= _last_health_and_arming_check + 100_ms) || _status_changed || nav_state_or_failsafe_changed) {
```

→ **実際の failsafe 発火は `COM_OF_LOSS_T` + 最大 100 ms 程度。**
`COM_OF_LOSS_T = 0.2` にしても発火は最大 0.3 秒後になる。

---

## 5. 遅延予算

制御ループ全体の遅延を段ごとに分解する。

```
  IMU/カメラ ──▶ VIO ──▶ MPC ──▶ DDS ──▶ uORB ──▶ 次のジャイロtick ──▶ モータ
     │            │        │        │        │           │
   ~1ms        10-30ms   <1ms    1-5ms    <1ms       0-2.5ms
                          ▲
                    feedback phase のみ
                    (preparation は先回り済み)
```

| 段 | 典型値 | 測定方法 |
|---|---|---|
| VIO 推定遅延 | 10〜30 ms | VIO の出力 timestamp とフレーム取得時刻の差 |
| **MPC feedback phase** | **< 1 ms** | `acado_feedbackStep()` 前後の計測 |
| MPC preparation | ~3 ms | 別スレッドなのでループ遅延には入らない |
| DDS 転送 | 1〜5 ms | ROS 2 側 publish 時刻と `uorb top` の受信時刻の差 |
| uORB → レート制御器 | 0〜2.5 ms | `IMU_GYRO_RATEMAX = 400` なら最大 2.5 ms 待つ |

**支配項は VIO の推定遅延である。** MPC の計算時間ではない。

`rate_mismatch.md` §3.3 で述べたとおり、RTI の preparation/feedback 分割によって
「計算時間 3.53 ms」は制御遅延に**寄与しない**。
観測が届いてから指令が出るまでは feedback phase の 1 ms 未満だけ。

### 実測方法

PX4 側:

```
nsh> uorb top vehicle_rates_setpoint vehicle_angular_velocity
```

ログ解析では `timestamp` (publish 時刻) ではなく `timestamp_sample` (センサ取得時刻) を使う。
`vehicle_rates_setpoint.timestamp` と、それを消費した
`vehicle_torque_setpoint.timestamp_sample` の差がループ内遅延になる。

> **注意**: uXRCE-DDS はセッション時刻オフセット (`time_offset_us`) で timestamp を変換している
> (`src/modules/uxrce_dds_client/uxrce_dds_client.cpp:75-103`)。
> ROS 2 側とフライトコントローラ側の時計は同期されているが、
> `vehicle_rates_setpoint.timestamp` を読んでいるコードは PX4 内に存在しないため、
> これは**ログ解析用**にしか使われない。

---

## 6. 失敗モードと対策

### 6.1 【最重要】陳腐化 setpoint の無限保持

**PX4 には `vehicle_rates_setpoint` の timeout が存在しない。**

`mc_rate_control` は最後に受け取った角速度と推力を 400 Hz で出し続ける
(`doc_jp/control_cascade.md` §4.4 参照)。

そして failsafe は**別トピック** `offboard_control_mode` を見ている
(`src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp:47-50`)。

**危険なシナリオ:**

> heartbeat を出すタイマースレッドは生きているが、MPC の計算スレッドがハングした。
> → `OffboardControlMode` は流れ続けるので failsafe は発火しない。
> → 機体は最後の角速度指令を実行し続ける。

**対策:**

1. **heartbeat と setpoint を同じコールバック / 同じスレッドから publish する**

   ```cpp
   void MpcNode::controlLoop()  // MPC のレートで回す
   {
     auto u = mpc_.run(state_, reference_);   // ここがハングすれば
     publishOffboardControlMode();            // heartbeat も止まる
     publishRatesSetpoint(u);
   }
   ```

   タイマーを分けると「heartbeat だけ生き残る」状態が作れてしまう。**分けないこと。**

2. **コンパニオン側 watchdog** — 解が期限内に出なければ自分で安全指令に切り替える

   ```cpp
   if (mpc_solve_age > kMaxSolveAge) {
     // ホバリング相当の安全指令、または heartbeat を止めて PX4 の failsafe に委ねる
   }
   ```

3. `COM_OF_LOSS_T` を短くする (§4 参照)

### 6.2 offboard 以外で setpoint を publish してしまう

uXRCE-DDS にはゲートがない (§1)。
`VehicleControlMode` を購読し、`flag_control_offboard_enabled` が false の間は publish を止める。

### 6.3 空中 disarm

`COM_DISARM_LAND = -1` を設定する (§4)。

### 6.4 `MPC_*` パラメータの取り違え

**PX4 の `MPC_*` は MultiCopter Position Controller であって Model Predictive Control ではない。**
`docs/en/contribute/notation.md` に明記されている。

`MPC_XY_VEL_P_ACC` などを「MPC のゲイン」と勘違いして触らないこと。
body_rate offboard では `mc_pos_control` 自体が動いていないので、これらは何の影響も持たない。
例外は `MPC_THR_HOVER` で、これは推力変換の基準として意味を持つ (§3.2)。

### 6.5 姿勢ループの帯域不足

MPC のレートが 20 Hz を下回ると姿勢ループの位相余裕が不足する。
対策は `rate_mismatch.md` §9 を参照 (予測入力列の補間、または attitude 経路への切り替え)。

---

## 7. 段階的な立ち上げ手順

いきなり実機で body_rate offboard を試さないこと。

### Step 1: SITL で配線を確認

```bash
make px4_sitl gz_x500
```

別ターミナルで:

```bash
MicroXRCEAgent udp4 -p 8888
```

パラメータ:

```
param set COM_DISARM_LAND -1
param set COM_OF_LOSS_T 0.5
param save
```

ROS 2 ノードを起動し、`uorb top` で `vehicle_rates_setpoint` が期待レートで届いているか確認する。

### Step 2: SITL でホバリング

MPC の参照を「その場でホバリング」に固定して、高度が保てるか見る。
ここで発散するなら推力変換 (§3.2) が間違っている。

### Step 3: SITL で軌道追従

論文と同じ円軌道 (1〜3 m/s) を試す。
ログで `vehicle_rates_setpoint` と `vehicle_angular_velocity` を重ね、追従を確認する。

### Step 4: 実機でホバリング (紐付き / 屋内)

- 先に Position モードで飛ばして `MPC_THR_HOVER` を較正する
- RC 送信機を必ず持ち、いつでもモードを抜けられるようにする
- `COM_OBL_RC_ACT` を確認しておく

### Step 5: 実機で軌道追従

速度を段階的に上げる。

---

## 8. 検証ログの見方

### 角速度追従

`vehicle_rates_setpoint.roll/pitch/yaw` と `vehicle_angular_velocity.xyz[0..2]` を重ねる。

- **階段状の setpoint に対して実測が滑らかに追従していれば正常。**
  ZOH の階段が見えること自体は問題ない (`rate_mismatch.md` §6)
- 実測が階段の各段で振動しているならレートゲインが高すぎる
- 追従が遅れているならレートゲインが低すぎる、または `IMU_DGYRO_CUTOFF` が低すぎる

### レート制御器の内部状態

`rate_ctrl_status` トピックに P/I/D 各項と積分値が入っている。
積分項が飽和していないか確認する。

### 実レートの確認

```
nsh> uorb top
```

`vehicle_angular_velocity` が `IMU_GYRO_RATEMAX` どおり出ているか、
`vehicle_rates_setpoint` が MPC のレートで届いているかを見る。

### 制御配分の飽和

`control_allocator_status.torque_setpoint_achieved` が false になっていないか。
false ならモータが飽和していて、指令どおりのトルクが出ていない。
`mc_rate_control` はこれを受けて anti-windup をかけている
(`MulticopterRateControl.cpp:196-216`)。

---

## 参考

- `doc_pampc/rate_mismatch.md` — レート整合性の理論的背景
- `doc_jp/control_cascade.md` — PX4 制御カスケードの詳細
- `docs/en/flight_modes/offboard.md` — offboard モードの公式仕様
- `docs/en/ros2/offboard_control.md` — ROS 2 offboard の実装例
- `src/modules/mc_raptor/README.md` — 外部制御器を接続した実例 (SITL 設定が参考になる)
