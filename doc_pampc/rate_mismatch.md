# PAMPC の指令レートと PX4 の角速度ループレートは整合するのか

## この文書が答える問い

PAMPC ([Falanga et al., IROS 2018](https://rpg.ifi.uzh.ch/docs/IROS18_Falanga.pdf) / [uzh-rpg/rpg_mpc](https://github.com/uzh-rpg/rpg_mpc)) は
機体状態を受け取り、**角速度 (body rate) と推力 (collective thrust)** を出力する。
これを PX4 に offboard で流し込んで自律飛行させたい。

ここで疑問が生じる。

> MPC なので指令が計算されるのは 10 Hz 程度。
> 一方 PX4 の制御カスケードでは角速度は 1000 Hz 程度で計算しなければならないはず。
> 問題ないのか?

**結論から言うと、この疑問には前提の誤りと、本質的に正しい懸念が両方含まれている。**

---

## 結論 (先出し)

1. **PX4 の内側ループは setpoint のレートに依存せず回り続ける。**
   `mc_rate_control` はジャイロ更新で駆動され、角速度 setpoint はゼロ次ホールドされる。
   setpoint が 10 Hz でも、レート PID は 400 Hz で最新の実測角速度に対して回る。
   → **角速度ループそのものは問題にならない。** これはカスケード制御の設計意図そのもの。

2. **「PX4 は 1000 Hz 必要」は不正確。**
   内側ループレートは `IMU_GYRO_RATEMAX` で決まり、**デフォルトは 400 Hz**。

3. **「PAMPC は 10 Hz」も論文の読み違い。**
   `dt = 0.1 s` は予測ホライズンの**離散化ステップ**であって制御周期ではない。
   論文は制御ループを **100 Hz** で回したと明記している。

4. **本当の論点は角速度ループではなく「姿勢ループ」。**
   PAMPC は PX4 の位置ループと姿勢ループの両方を置き換える。
   角速度指令は姿勢に対して開ループ積分なので、MPC の更新レートが姿勢ループの帯域を直接律速する。
   100 Hz なら十分、10 Hz だと位相余裕を食い潰す。

5. **そして実装上の最大リスクはレートではなく「陳腐化 setpoint の保持」。**
   PX4 には `vehicle_rates_setpoint` の timeout が存在しない。
   MPC がハングしても最後の角速度指令を無限に出し続ける。

---

## 1. PAMPC が PX4 に渡すもの

論文 §III-B, Eq.(3) の状態方程式:

$$
\dot{\mathbf{p}}_{WB} = \mathbf{v}_{WB}, \quad
\dot{\mathbf{v}}_{WB} = {}_W\mathbf{g} + \mathbf{q}_{WB} \odot \mathbf{c}, \quad
\dot{\mathbf{q}}_{WB} = \frac{1}{2}\Lambda(\mathbf{\Omega}_B)\cdot\mathbf{q}_{WB}
$$

状態と入力は:

$$
\mathbf{x} = [\mathbf{p}_{WB}, \mathbf{v}_{WB}, \mathbf{q}_{WB}], \qquad
\mathbf{u} = [c, \mathbf{\Omega}_B^\top]^\top
$$

つまり入力は **質量正規化 collective thrust $c$ + 機体角速度 $\mathbf{\Omega}_B$**。

なぜ MPC がモータトルクではなく角速度を出すのか。
クアッドロータは劣駆動 (underactuated) で、並進運動は姿勢を介してしか制御できない。
そして姿勢ダイナミクスはロータの応答時定数に支配され、並進ダイナミクスより 1〜2 桁速い。
この**時間スケール分離**を利用して、遅い部分 (並進 + 姿勢の参照生成) を MPC に、
速い部分 (角速度追従) を専用の高レート制御器に分担させるのが標準的な設計である。

**PAMPC は最初からこの分担を前提に設計されている。** 次節以降でそれを確認する。

---

## 2. 「10 Hz」の出所 — 離散化ステップと制御周期の混同

論文の該当箇所を確認する。

### 論文 §VI-D "PAMPC Parameters"

> We chose a discretization of $dt = 0.1\,\mathrm{s}$ and a time horizon of $t_h = 2\,\mathrm{s}$.

ホライズン 2 秒を 0.1 秒刻みで離散化する。ノード数 $N = t_h/dt = 20$。
**これは予測グリッドの粗さであって、制御出力のレートではない。**

### 論文 §V-A "Experimental Setup"

> As discretization step, we chose $dt = 0.1\,\mathrm{s}$ with a time horizon of $t_h = 2\,\mathrm{s}$
> and **ran one iteration step in each control loop with a frequency of 100 Hz**.
> Therefore, the iteration ran roughly **10× faster than the discretization time**,
> resulting in small deviations of the predicted state vector between iterations and facilitating convergence.

**制御ループは 100 Hz。** 離散化ステップより 10 倍速い。

### 論文 §IV

> To achieve good approximations, it is important to run these iterations
> **significantly faster than the discretization time** of the problem
> and to keep the previous solution as initialization trajectory of the next optimization.

「離散化ステップより十分速く回すことが重要」と明示的に述べている。

### 論文 §VI-E "Computation Time"

> our PAMPC requires on average **3.53 ms**. […] the maximal execution time always stays below **5 ms**.

100 Hz (= 10 ms 周期) に対して計算時間 3.53 ms。十分間に合っている。

**まとめ:**

| | 値 | 意味 |
|---|---|---|
| 離散化ステップ $dt$ | 0.1 s | 予測ホライズンのノード間隔。20 ノード × 0.1 s = 2 s |
| **制御周期** | **0.01 s (100 Hz)** | 再最適化して指令を出す間隔 |
| 計算時間 | 3.53 ms (最大 < 5 ms) | 1 周期あたりの実行時間 |

「10 Hz」に見えた数字は $dt$ である。**PX4 に届く角速度指令は 100 Hz で更新される。**

---

## 3. Real-Time Iteration (RTI) スキーム

「100 Hz で最適化を回す」と聞くと非現実的に思えるが、RTI という仕掛けがある。
`rpg_mpc` の実装コードで確認できる内容をまとめる。

### 3.1 教科書 MPC との違い: 1 周期あたり SQP は 1 反復だけ

教科書的な MPC は、各制御周期で非線形最適化を**収束するまで**解く (SQP を残差が小さくなるまで反復)。
100 Hz には到底間に合わない。

RTI は **1 制御周期あたり SQP をきっかり 1 反復だけ**回す。

1. 前回の解の周りで線形化する (ウォームスタート)
2. QP を 1 回だけ qpOASES で解く
3. 出てきた $u_0$ を即座に出力する
4. 10 ms 後、また 1 反復

**どの瞬間においても収束していない。**
収束は時間軸方向に起こる (convergence on the fly)。
10 ms では状態がわずかしか動かないので前回解が極めて良い初期値になり、
反復列が「動いている最適解」を追尾し続ける。

論文 §V-A の "resulting in small deviations of the predicted state vector between iterations and
facilitating convergence" はまさにこれを指している。

**これが「離散化ステップより十分速く回す必要がある」理由でもある。**
問題が変化する速さに対して反復が十分速くないと、追尾が破綻する。

### 3.2 「1 step 進む」の step は 0.1 s ではなく 0.01 s

ここが最初の疑問に直結する。**"step" が 2 種類ある。**

1 制御周期で進むのは 10 ms、つまり**ノード 1 個分の 1/10 だけ**である。
ホライズンをノード単位でシフトしているわけではない。

実際 `rpg_mpc` の `src/mpc_wrapper.cpp` には `acado_shiftStates()` / `acado_shiftControls()` の
呼び出しが**存在しない**。
前回解はそのままウォームスタートとして残し、`acado_initial_state_` に最新推定値を上書きして、
参照軌道を現在時刻で再サンプルするだけ。
結果としてホライズンは $[t,\ t+2\mathrm{s}]$ として連続的にスライドする。

### 3.3 preparation / feedback の 2 相分割 — 100 Hz を可能にしている肝

SQP 1 反復は「重い部分」と「軽い部分」に分けられる。

| フェーズ | 処理内容 | 新しい観測が必要か | 時間 |
|---|---|---|---|
| **preparation** | 力学の線形化、感度計算、QP の condensing | **不要** | ~3 ms |
| **feedback** | 初期条件を差し込んで QP を解く | 必要 | < 1 ms |

`rpg_mpc` の `MpcWrapper::update()`:

```cpp
acado_initial_state_ = state.template cast<float>();
acado_feedbackStep();                  // 観測が来てから、ここだけ
acado_is_prepared_ = false;
if (do_preparation) {
  acado_preparationStep();             // 次周期のぶんを先回りして済ませる
  acado_is_prepared_ = true;
}
```

preparation は**別スレッド**で走る:

```cpp
preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);
```

タイムライン:

```
        観測到着            観測到着
           │                  │
  ─────────┼──────────────────┼──────────  時間
           │                  │
           ├ feedback (<1ms)  │
           │  → u_0 出力      │
           │                  │
           └── preparation ───┘   (別スレッド、次周期用に先回り)
                 (~3ms)
```

**観測から指令出力までの実質的な遅延は feedback phase の分 (< 1 ms) だけ。**
計算時間 3.53 ms と制御遅延は別物である。この区別は遅延予算を組むときに効いてくる。

出力は予測入力列の先頭:

```cpp
return updateControlCommand(predicted_states_.col(0),
                            predicted_inputs_.col(0),   // ← u_0 = [c, Ω_B]
                            call_time);
```

初回のみ `solve_from_scratch_` で完全収束させる (`"Solving MPC with hover as initial guess."`)。
以降はずっと 1 反復ずつ。

---

## 4. PX4 側 — 「1000 Hz 必要」の実際

PX4 の内側ループレートは `IMU_GYRO_RATEMAX` で決まる。

```yaml
IMU_GYRO_RATEMAX:
  description:
    short: Gyro control data maximum publication rate (inner loop rate)
    long: |-
      The maximum rate the gyro control data (vehicle_angular_velocity) will be
      allowed to publish at. This is the loop rate for the rate controller and outputs.
  values: {100, 250, 400, 800, 1000, 2000}
  default: 400
```

**デフォルトは 400 Hz** であって 1000 Hz ではない。
1 kHz にしたければ明示的に設定する (再起動が必要)。

カスケード各段のデフォルトレート:

| 段 | 実レート | 決定パラメータ |
|---|---|---|
| 角速度制御 | **400 Hz** | `IMU_GYRO_RATEMAX` |
| 姿勢制御 | 200 Hz | `IMU_INTEG_RATE` |
| 位置制御 | ~100 Hz | `EKF2_PREDICT_US` |

詳細は `docs_ja/04_control_cascade.md` を参照。

---

## 5. なぜ外側ループが遅くてよいのか

制御系の議論では、**2 つのまったく別のレート**が混同されやすい。

| | 何を決めるか | 誰が決めるか |
|---|---|---|
| **フィードバックループのレート** | 外乱抑圧能力、閉ループ帯域、安定性 | 内側ループ (PX4 の 400 Hz) |
| **参照 (setpoint) の更新レート** | 追従したい軌道の帯域 | 外側ループ (MPC の 100 Hz) |

「角速度は 1 kHz で計算する必要がある」という主張が正しいのは**前者**についてである。
ロータと機体の回転ダイナミクスは速く、減衰が小さいので、
これを安定化するフィードバックループは速く回す必要がある。

しかし**参照値の更新レート**にその要求は及ばない。
参照は Nyquist の意味で「指令したい軌道の帯域」の 2 倍以上あればよく、
実用上は閉ループ帯域の 10〜20 倍が目安になる。

PAMPC を接続しても、PX4 のレートループは 400 Hz のまま何も変わらない。
**外乱抑圧は引き続き内側ループが担い、MPC は参照を与えるだけ。**

### ゼロ次ホールドの実効遅延

setpoint が周期 $T_s$ で更新されて間はホールドされる場合、
平均的に $T_s/2$ の遅延が入る。周波数 $f$ における位相遅れは:

$$
\phi = 2\pi f \cdot \frac{T_s}{2}
$$

これが第 8 節で効いてくる。

---

## 6. PX4 コードでの裏付け

### 6.1 setpoint はラッチされる

`src/modules/mc_rate_control/MulticopterRateControl.cpp:181-186`:

```cpp
	} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
		_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
		_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
		_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
		_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
	}
```

`update()` は新しいサンプルが届いたときだけ true を返す。
`_rates_setpoint` はクラスメンバで、ヘッダにコメントがある (`MulticopterRateControl.hpp:125`):

```cpp
	// keep setpoint values between updates
```

そして PID 本体は setpoint の更新有無に関係なく毎ジャイロサンプル走る (`:219-220`):

```cpp
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
```

**MPC が 100 Hz なら、同じ setpoint が 4 回連続で使われる (400/100)。
10 Hz なら 40 回連続。いずれの場合もレート PID は 400 Hz で回り続ける。**

### 6.2 微分キックが起きない構造になっている

`src/lib/rate_control/rate_control.cpp:78`:

```cpp
	// PID control with feed forward
	Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
```

D 項は**誤差の微分ではなく実測角加速度**を使っている。

一般的な誤差微分型 PID なら、階段状の setpoint が $d(e)/dt$ に巨大なインパルスを生み、
トルク指令が周期的に跳ねる。低レート setpoint では致命的になりうる。

PX4 は測定値微分型なので**原理的にこれが起こらない**。
低レート・階段状の setpoint に対して構造的に耐性がある。

---

## 7. RPG 側も同じ構成になっている

決定的な事実として、**PAMPC 自身が「高レート角速度ループは別のプロセッサが担当する」前提で設計されている。**

同じ研究室の低レベル制御器の論文
[Faessler et al., RA-L 2017](https://rpg.ifi.uzh.ch/docs/RAL17_Faessler.pdf) §III:

> Due to a hardware architecture with **two processing units for high-level and low-level control**
> on our quadrotors, we split the controllers such that
> **the high-level controller computes desired body rates and the low-level controller tracks them**.

> The inputs to this controller are the desired body rates $\omega_{des}$ and the desired
> mass normalized collective thrust $c_{des}$ which we assume are given from a high-level position controller.

そして `rpg_quadrotor_control` の Wiki (Overview and Concepts):

> The low-level controller receives the control commands from the high-level controller through
> a so called bridge and controls either body rates or attitude.
> In our case this is implemented in the **Betaflight Firmware**.

Betaflight の角速度 PID ループは kHz 級で回る。

つまり論文の実験構成は:

```
  PAMPC (100 Hz, Snapdragon Flight)  ──[c, Ω_B]──▶  Betaflight 角速度ループ (kHz級)
```

これを PX4 に置き換えると:

```
  PAMPC (100 Hz, コンパニオン)  ──[c, Ω_B]──▶  PX4 mc_rate_control (400 Hz)
```

**構造的に 1:1 の置き換えである。** アーキテクチャ上まったく想定内の使い方であり、
「MPC が遅いのに PX4 に繋いで大丈夫か」という懸念は、この対応関係を見れば解消する。

---

## 8. 本当に注意すべき点 — 姿勢ループの帯域

ここからが実質的な論点である。

`offboard_control_mode.body_rate` を選ぶと、PX4 側の設定は
`src/modules/commander/ModeUtil/setpoint_types.cpp:85-88`:

```cpp
	case SetpointType::Rates:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		break;
```

`flag_control_attitude_enabled` が立たない。
→ **`mc_att_control` (200 Hz) は完全に停止し、姿勢ループを閉じるのは MPC になる。**

つまり:

| ループ | 通常の PX4 | PAMPC 接続時 |
|---|---|---|
| 位置 | `mc_pos_control` ~100 Hz | **MPC** |
| 姿勢 | `mc_att_control` 200 Hz | **MPC** ← ここが問題 |
| 角速度 | `mc_rate_control` 400 Hz | `mc_rate_control` 400 Hz (変化なし) |

### なぜ姿勢だけ特別なのか

**角速度指令は姿勢に対して開ループ積分になるから。**

$$
\dot{\mathbf{q}} = \frac{1}{2}\Lambda(\mathbf{\Omega})\cdot\mathbf{q}
$$

角速度を指令している間、姿勢はその積分として自由に流れていく。
姿勢の誤差を補正する機会は **MPC が次に解を出すときだけ**である。
その間、姿勢に関してはフィードバックが一切かかっていない。

対照的に角速度は、実測角速度に対して 400 Hz でフィードバックがかかり続けている。

### 位相遅れで見た定量評価

ZOH の実効遅延 $T_s/2$ による位相遅れを、クアッドの姿勢ループの典型的なクロスオーバー周波数
$f = 3\,\mathrm{Hz}$ で評価する。

$$
\phi = 2\pi f \cdot \frac{T_s}{2} \quad\Rightarrow\quad \phi[\mathrm{deg}] = 360 \cdot f \cdot \frac{T_s}{2}
$$

| MPC レート | $T_s$ | 3 Hz での位相遅れ | 評価 |
|---|---|---|---|
| **100 Hz** | 10 ms | **5.4°** | 無視できる |
| 50 Hz | 20 ms | 10.8° | 許容範囲 |
| 20 Hz | 50 ms | 27° | 位相余裕を大きく食う |
| **10 Hz** | 100 ms | **54°** | **設計上の位相余裕 (45〜60°) をほぼ全部消費** |

**10 Hz では姿勢ループの位相余裕が事実上残らない。**
穏やかなホバリングなら飛ぶかもしれないが、外乱が入ると振動する。
アグレッシブな機動は成立しない。

これがユーザーの元々の直感 —「レートが足りないのでは」— の**正しい部分**である。
ただし懸念の対象は角速度ループではなく姿勢ループだった、ということになる。

---

## 9. どうしても 10 Hz 程度しか出せない場合の対策

計算資源の制約で MPC の解算を高レートで回せない場合、以下の順で検討する。

### 対策 A (最推奨): 解算レートと出力レートを分離する

**MPC はホライズン全体の入力列 $u_0, u_1, \dots, u_{N-1}$ を持っている。**
「解くのは 10 Hz、publish は 100〜200 Hz」にできる。

```
  MPC 解算 (10 Hz)
       │
       ├── u_0 … u_19  (0.1 s 刻み、2 秒ぶんの予測入力列)
       │
       ▼
  補間して高レート出力 (100〜200 Hz)  ──▶  PX4 VehicleRatesSetpoint
```

実装:

1. MPC が解を出すたびに、予測入力列 $\{u_i\}$ とその時刻タグを保持する
2. 別スレッド (または ROS 2 タイマー) を 100〜200 Hz で回す
3. 現在時刻に対応する $u(t)$ を線形補間して `VehicleRatesSetpoint` に publish する

**ほぼ追加計算コストなしで姿勢ループの位相遅れが解消する。**
ZOH の階段が滑らかな軌道に置き換わるため、位相遅れは $T_s/2 = 100$ ms 相当から
publish 周期の $T_s/2 = 5$ ms 相当に落ちる。

注意点として、これは**予測が当たっている前提**の補外である。
外乱で実際の状態が予測から離れると補間値は最適でなくなるが、
「古い解をホールドし続ける」よりは常に良い。

### 対策 B: `body_rate` ではなく `attitude` で渡す

`offboard_control_mode.attitude` + `VehicleAttitudeSetpoint` に切り替える。

```cpp
	case SetpointType::Attitude:
		control_mode.flag_control_allocation_enabled = true;
		control_mode.flag_control_rates_enabled = true;
		control_mode.flag_control_attitude_enabled = true;   // ← mc_att_control が動く
		break;
```

こうすると **PX4 の `mc_att_control` (200 Hz) が姿勢ループを閉じる。**
MPC は姿勢の「参照」を与えるだけになる。

**姿勢参照は角速度参照と違い、ホールドしてもドリフトしない。**
角速度参照は積分器の入力なので保持すると姿勢が流れていくが、
姿勢参照は保持しても姿勢がその値に留まるだけである。
低レート化に対する耐性が構造的に違う。

MPC の解から姿勢参照を作るには、予測された推力ベクトル方向と yaw 参照からクォータニオンを合成する
(PX4 内部にも同じ変換が `ControlMath::thrustToAttitude()` として存在する)。

トレードオフ:

| | `body_rate` 経路 | `attitude` 経路 |
|---|---|---|
| MPC の最適性 | そのまま反映される | 姿勢に射影する分だけ失われる |
| 低レート耐性 | 弱い (開ループ積分) | 強い (PX4 が 200 Hz で補間) |
| PX4 のゲイン調整 | レートゲインのみ | 姿勢 P ゲインも要調整 |
| アグレッシブ機動 | 高レートなら有利 | 姿勢制御器の帯域が上限になる |
| 推奨レート | ≥ 50 Hz | ≥ 10 Hz でも成立しうる |

**10 Hz 確定なら対策 B、可能なら対策 A を優先する。**

### 対策 C: `IMU_GYRO_RATEMAX` を上げる

アグレッシブな機動を狙うなら 800 または 1000 Hz にする。
ただし**これは内側ループの話であり、姿勢ループの帯域不足は解決しない。**
対策 A / B の代わりにはならない。

### 対策 D: 計算時間の実測から見直す

論文の PAMPC は Snapdragon Flight (2.26 GHz ARM、4 コア中 3 コアは VIO が使用) で
**平均 3.53 ms、最大 5 ms** である。

最近のコンパニオンコンピュータ (Jetson Orin Nano、Raspberry Pi 5、x86 ミニ PC など) は
これより大幅に速い。
「10 Hz しか出ない」と決める前に、実際に ACADO のコード生成を動かして計測することを勧める。
RTI は 1 反復しか回さないので、想像より遥かに軽い。

---

## 10. レートより大きい実装リスク — 陳腐化 setpoint の保持

最後に、レートの議論より優先度の高い安全上の問題を挙げておく。

**PX4 には `vehicle_rates_setpoint` の陳腐化チェックが存在しない。**
`src/` 全体を検索しても、このトピックの timestamp を検査しているコードはない。

つまり **MPC が落ちても、レート制御器は最後の角速度と推力を 400 Hz で無限に出し続ける。**

唯一の保護は commander の offboard failsafe だが、これは**別のトピック**を見ている。
`src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp:47-54`:

```cpp
	if (_offboard_control_mode_sub.copy(&offboard_control_mode)) {

		bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
				      + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);
```

見ているのは `offboard_control_mode.timestamp` であって `vehicle_rates_setpoint` ではない。

**帰結として、次の状況では failsafe が永久に発火しない:**

> heartbeat (`OffboardControlMode`) を出すスレッドは生きているが、
> MPC の計算スレッドがハングして `VehicleRatesSetpoint` の更新が止まった場合。

機体は最後の角速度指令を実行し続ける。それが大きなロール指令だった場合は即墜落する。

**対策:**

1. **heartbeat と setpoint を同じスレッドから出す。** MPC が止まれば heartbeat も止まる構造にする
2. コンパニオン側に watchdog を実装し、解が期限内に出なければ自分で安全指令 (ホバリング相当) に切り替える
3. `COM_OF_LOSS_T` (default 1.0 s) を短くする。ただし health check の粒度が 100 ms
   (`Commander.cpp:2050`) なので、実際の発火は設定値 + 最大 100 ms 程度になる

詳細と設定手順は `doc_pampc/integration_guide.md` を参照。

---

## まとめ

| 当初の想定 | 実際 |
|---|---|
| PAMPC の指令は 10 Hz | **100 Hz** (0.1 s は離散化ステップ) |
| PX4 の角速度ループは 1000 Hz 必要 | デフォルト **400 Hz** (`IMU_GYRO_RATEMAX` で設定) |
| レート不整合で角速度制御が破綻する | **破綻しない。** setpoint は ZOH され、PID は 400 Hz で回り続ける |
| — | 真の論点は **姿勢ループの帯域**。10 Hz だと位相余裕をほぼ失う |
| — | 真の最大リスクは **陳腐化 setpoint の無限保持** |

**PAMPC → PX4 は、論文自身の実験構成 (PAMPC → Betaflight) と構造的に同一である。**
100 Hz 前後で回せるなら、アーキテクチャ上の問題はない。

---

## 参考文献

- [PAMPC: Perception-Aware Model Predictive Control for Quadrotors](https://rpg.ifi.uzh.ch/docs/IROS18_Falanga.pdf) — Falanga, Foehn, Lu, Scaramuzza, IROS 2018
- [uzh-rpg/rpg_mpc](https://github.com/uzh-rpg/rpg_mpc) — 実装 (ACADO + qpOASES)
- [Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight](https://rpg.ifi.uzh.ch/docs/RAL17_Faessler.pdf) — Faessler, Falanga, Scaramuzza, RA-L 2017
- [uzh-rpg/rpg_quadrotor_control Wiki](https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/Overview-and-Concepts)
- `docs_ja/04_control_cascade.md` — PX4 制御カスケードの詳細（全体解説は `docs_ja/README.md`）
- `doc_pampc/integration_guide.md` — 実装手順
