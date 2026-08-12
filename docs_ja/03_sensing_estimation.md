# 03. センシングと状態推定

[← 前へ: 02. uORB と実行モデル](02_uorb_runtime.md) | [目次](README.md) | [次へ: 04. 制御カスケード →](04_control_cascade.md)

---

自律制御の前提は「**機体が今どこにいて、どちらを向いていて、どう動いているか**」を知ることです。
この章では生センサ値がどう処理され、EKF2 でどう融合され、
制御が使う `vehicle_attitude` / `vehicle_local_position` になるかを追います。

---

## 3.1 推定パイプライン全体図

```
 物理センサ (SPI/I2C/UART)
      │
      ▼
┌──────────────────────────────────────────────┐
│ ドライバ (src/drivers/imu/…, src/drivers/gnss/…) │
│  publish: sensor_accel, sensor_gyro,          │
│           sensor_mag, sensor_baro, sensor_gps │
└──────────────────────────────────────────────┘
      │
      ▼
┌──────────────────────────────────────────────┐
│ sensors モジュール (src/modules/sensors/)      │
│  ・キャリブレーション適用・回転補正             │
│  ・多重センサの投票／フェイルオーバ              │
│  ・IMU 積分（コーニング/スカリング補正）         │
│                                               │
│  vehicle_imu           ← IMU 積分値（Δv, Δθ）  │
│  vehicle_angular_velocity ← ★ 制御用ジャイロ     │
│  vehicle_acceleration                          │
│  vehicle_air_data      ← 気圧高度               │
│  vehicle_magnetometer                          │
│  vehicle_gps_position                          │
│  sensor_combined       ← ログ／互換用まとめ      │
└──────────────────────────────────────────────┘
      │
      ▼
┌──────────────────────────────────────────────┐
│ ekf2 (src/modules/ekf2/)                      │
│  24 誤差状態の拡張カルマンフィルタ               │
│                                               │
│  ・EKF コア: Δθ/Δv で予測 → 各センサで補正       │
│    ただし融合するのは「過去の時刻」（遅延補償）      │
│  ・出力予測器: そこから現在時刻まで IMU で外挿      │
│    → 2 段構造。詳細は 3.4 節                     │
│                                               │
│  vehicle_attitude        ← ★ 姿勢（クォータニオン）│
│  vehicle_local_position  ← ★ ローカル NED 位置速度 │
│  vehicle_global_position ← 緯度経度高度          │
│  vehicle_odometry        ← 外部連携用            │
│  estimator_status(_flags) ← 健全性              │
└──────────────────────────────────────────────┘
      │
      ▼
   制御カスケードへ（04 章）
```

**重要**: 制御に使われるのは `vehicle_angular_velocity`（角速度制御用）と
`vehicle_attitude` / `vehicle_local_position`（姿勢・位置制御用）の 3 つです。
生センサ値（`sensor_gyro` など）は制御には使いません。

> [!NOTE]
> **この章で押さえるべき 3 点**（いずれも誤解が多い箇所です）:
>
> 1. **加速度計は「加速度」ではなく比力を測る**（重力込み）→ [3.2](#32-生センサは何を測っているのか)
> 2. **センサ → 状態の向きは微分ではなく積分**（二重積分の誤差を観測で押さえる）→ [3.4](#向きは積分-微分ではない)
> 3. **EKF2 は加速度を状態として推定していない**。`ax/ay/az` は生値の加工 → [3.4](#3-つの出力の出自は違う)

---

## 3.2 生センサは何を測っているのか

**ここを誤解すると EKF2 の設計が理解できません。** 特に加速度計です。

| センサ | **実際に測っている量** | `sensors` が EKF に渡す形 | 注意点 |
| --- | --- | --- | --- |
| ジャイロ | 角速度 [rad/s] | **`delta_angle`**（Δθ, 積分増分） | バイアスがドリフトする |
| 加速度計 | **比力**（specific force）[m/s²] | **`delta_velocity`**（Δv, 積分増分） | **重力を含む。位置の 2 階微分ではない** |
| GNSS | **位置 と 速度の両方** | `vehicle_gps_position` | 遅延が大きい（100〜200 ms） |
| 気圧計 | 気圧 → 高度 | `vehicle_air_data` | 絶対値は不正確、相対変化は速い |
| 磁気計 | 磁場ベクトル → 方位 | `vehicle_magnetometer` | 機体の電流・鉄材で汚染される |
| 距離計 / フロー | 対地高度 / 対地速度 | `distance_sensor` / `vehicle_optical_flow` | 屋内・低高度用 |

### 加速度計は「加速度」を測っていない ★

加速度計が測るのは **比力（proper acceleration）** です。

- **静止しているとき** → 上向きに **1 g** を示す（重力に抗して支えられているため）
- **自由落下しているとき** → **0** を示す（何にも支えられていないため）

つまり「機体が動いた量」ではなく「機体が重力以外から受けた力」を測っています。
だから EKF は **重力を明示的に足し戻す**必要があります
（`src/modules/ekf2/EKF/output_predictor/output_predictor.cpp`）:

```cpp
// バイアスを引いた Δv を、姿勢を使って機体座標 → NED へ回転
Vector3f delta_vel_earth{_R_to_earth_now * delta_velocity_corrected};

// 加速度計は重力を含んで測るので、重力分を補正する
delta_vel_earth(2) += _gravity * delta_velocity_dt;
```

この 1 行がないと、静止していても「上向き 1 g で加速している」と解釈され、
速度と位置が際限なく発散します。

### なぜ「増分（Δθ, Δv）」で渡すのか

`sensors` モジュールが EKF に渡すのは瞬時値ではなく **積分増分**です
（`msg/VehicleImu.msg`）:

```
float32[3] delta_angle          # 積分期間中の Δθ [rad]（機体 FRD）
float32[3] delta_velocity       # 積分期間中の Δv [m/s]（機体 FRD）
uint32 delta_angle_dt           # 積分期間 [µs]
uint32 delta_velocity_dt        # 積分期間 [µs]
```

**ストラップダウン INS の標準的な形式**です。理由は 2 つあります。

1. **情報を捨てない** — IMU は 1 kHz で読めますが EKF は 100 Hz で回ります。
   単純に間引くと間のサンプルが失われますが、積分してから渡せば
   その期間の運動が全部含まれます（`Integrator` クラス、`src/modules/sensors/Integrator.hpp`）
2. **積分がそのまま状態更新になる** — EKF 側は Δθ / Δv を足し込むだけで済みます

> [!NOTE]
> **`vehicle_imu`（Δθ / Δv）と `vehicle_angular_velocity`（角速度 / 角加速度）は別物です。**
> 前者は EKF2 への入力、後者は角速度制御への入力です。
> 同じジャイロから作られますが、用途が違うのでフィルタ処理も別々です。

---

## 3.3 `sensors` モジュールの仕事

`src/modules/sensors/` は「生センサ → 制御が使える形」への橋渡しをします。

### サブモジュール構成

| ディレクトリ | 出力トピック | 役割 |
| --- | --- | --- |
| `vehicle_imu/` | `vehicle_imu`, `vehicle_imu_status` | ジャイロ・加速度の積分（Δθ, Δv） |
| `vehicle_angular_velocity/` | `vehicle_angular_velocity` | **制御用の角速度**（フィルタ済み） |
| `vehicle_acceleration/` | `vehicle_acceleration` | 制御用の加速度 |
| `vehicle_air_data/` | `vehicle_air_data` | 気圧高度・温度 |
| `vehicle_magnetometer/` | `vehicle_magnetometer` | 磁気（複数センサ統合） |
| `vehicle_gps_position/` | `vehicle_gps_position` | GNSS（複数受信機からブレンド） |
| `vehicle_optical_flow/` | `vehicle_optical_flow` | オプティカルフロー |
| `data_validator/` | — | センサ投票・異常検知の共通ロジック |

### `vehicle_angular_velocity` が特別な理由

`src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp` は
制御ループの品質を直接決める重要モジュールです。処理内容:

1. 選択中のジャイロ（`sensor_selection` で決まる）を購読
2. EKF2 が推定したジャイロバイアス（`estimator_sensor_bias`）を減算
3. **ノッチフィルタ** — プロペラ回転由来の振動を除去
   - 静的ノッチ（`IMU_GYRO_NF0_FRQ` 等）
   - 動的ノッチ — ESC の回転数（`esc_status`）や FFT 解析（`sensor_gyro_fft`）に追従
4. **ローパスフィルタ**（`IMU_GYRO_CUTOFF`）
5. 角加速度の算出（D 項に使う）
6. `vehicle_angular_velocity` として発行 → **`mc_rate_control` を駆動**

> [!IMPORTANT]
> **機体が振動する / モータが熱くなる場合、まずここのフィルタ設定を疑ってください。**
> ジャイロノイズが D 項で増幅され、モータが高周波で振動します。
> 関連パラメータ: `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_GYRO_NF0_*`, `IMU_GYRO_DNF_EN`

### センサの多重化とフェイルオーバ

IMU が複数ある機体では、`sensors` モジュールが投票（voting）を行い、
異常なセンサを除外します（`voted_sensors_update.cpp`）。
どれが選ばれているかは `sensor_selection` トピックで分かります。

```sh
listener sensor_selection
listener vehicle_imu_status -i 0
```

さらに EKF2 自体も複数インスタンス動作でき（`EKF2_MULTI_IMU`）、
`EKF2Selector`（`src/modules/ekf2/EKF2Selector.cpp`）が最良の推定器を選びます。

---

## 3.4 EKF2 — 状態推定の中核

`src/modules/ekf2/` が PX4 の標準推定器です。
`EKF2.cpp` が uORB との入出力を担当し、`EKF/` 配下がフィルタ本体です。

### 状態ベクトル

`src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h` に定義があります。

| 状態 | 次元 | 内容 |
| --- | --- | --- |
| `quat_nominal` | 4 | 姿勢クォータニオン（NED → body） |
| `vel` | 3 | NED 速度 [m/s] |
| `pos` | 3 | NED 位置 [m] |
| `gyro_bias` | 3 | ジャイロバイアス [rad/s] |
| `accel_bias` | 3 | 加速度計バイアス [m/s²] |
| `mag_I` | 3 | 地球磁場（NED） |
| `mag_B` | 3 | 機体磁場バイアス |
| `wind_vel` | 2 | 風速（水平） |

誤差状態は 24 次元（クォータニオンが 3 次元の回転誤差になるため）。

> [!NOTE]
> **バイアスが状態に含まれている**のが重要です。
> だから PX4 は起動後しばらく静止させるとジャイロバイアスが収束し、姿勢が安定します。
> 「起動直後に動かすとドリフトする」のはこのためです。

> [!IMPORTANT]
> **速度 `vel` は位置 `pos` を微分して求めているのではありません。**
> 両者は独立した状態量で、それぞれ観測で直接補正されます。
>
> - **予測**: 加速度計を積分して速度を進める
> - **補正**: GNSS の速度観測、オプティカルフロー、対気速度などで直接更新する
>   （`src/modules/ekf2/EKF/velocity_fusion.cpp` の `Ekf::fuseVelocity()`）
>
> 位置を数値微分すると、位置推定に乗ったノイズが微分で増幅されて
> 制御に使えない信号になります。EKF が速度を状態として持つことで、
> **位置よりも滑らかで応答の速い速度**が得られます。
> これが位置制御で速度 PID を主役にできる理由です
> （[04 章](04_control_cascade.md#_vel_dot-はどこから来るのか)）。

### 向きは「積分」— 微分ではない

センサから状態を作る向きを間違えないでください。**積分**です。

```
   Δθ（ジャイロ）──────────────────────────積分──→ 姿勢
                                                    │
                                                    │ この姿勢で回転させる
                                                    ▼
   Δv（加速度計）──[機体→NED へ回転]──[重力を足し戻す]──積分──→ 速度 ──積分──→ 位置
```

`OutputPredictor::calculateOutputStates()` の実装がそのままこの順序です:

```cpp
// ① バイアスを引いた Δθ で姿勢を進める
const Quatf dq(AxisAnglef{delta_angle_corrected});
_output_new.quat_nominal = _output_new.quat_nominal * dq;
_output_new.quat_nominal.normalize();

// ② その姿勢で Δv を NED へ回転し、重力を補正する
_R_to_earth_now = Dcmf(_output_new.quat_nominal);
Vector3f delta_vel_earth{_R_to_earth_now * delta_velocity_corrected};
delta_vel_earth(2) += _gravity * delta_velocity_dt;

// ③ 速度に足し込む
const Vector3f vel_last(_output_new.vel);
_output_new.vel += delta_vel_earth;

// ④ 台形積分で位置に足し込む
const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (delta_velocity_dt * 0.5f);
_output_new.pos += delta_pos_NED;
```

**加速度 → 速度 → 位置 の二重積分**なので、
加速度計のわずかなバイアスやノイズが時間とともに二乗で溜まります。
数秒で数メートル、数十秒で数十メートルずれます。
**この累積誤差を押さえ込むのがカルマン更新（補正）の役割**です。

| 誤差の抑え込み | 使う観測 |
| --- | --- |
| 位置のドリフト | GNSS 位置、外部Vision 位置、距離計（高度） |
| 速度のドリフト | **GNSS 速度**、オプティカルフロー、対気速度 |
| 姿勢の傾き誤差 | 加速度計（長期的には重力方向を示すため） |
| 方位のドリフト | 磁気計、GNSS の進行方向 |
| バイアスそのもの | 上記すべて（バイアスも状態なので同時に推定される） |

> [!IMPORTANT]
> **「位置を微分して速度を作る」処理は PX4 のどこにも存在しません。**
> 速度は加速度の積分（予測）と GNSS 速度などの観測（補正）から作られます。
> だから位置よりも滑らかで、位置制御で速度 PID を主役にできます
> （[04 章](04_control_cascade.md#_vel_dot-はどこから来るのか)）。

### 2 段構造 — EKF コアと出力予測器は並列に走る

EKF2 で最も特徴的な設計です。**直列のパイプラインではありません。**

```
        vehicle_imu（Δθ, Δv）    〜1 kHz
                 │
        ┌────────┴─────────────────────────────────┐
        │                                          │
        ▼ 毎サンプル、常に走る                        ▼ ダウンサンプルして貯める
┌──────────────────────────┐        ┌──────────────────────────────┐
│ 出力予測器                 │        │ IMU ダウンサンプラ + 遅延バッファ  │
│ OutputPredictor           │        │ バッファ長 = EKF2_DELAY_MAX     │
│                           │        │            （既定 200 ms）      │
│ Δθ/Δv を「現在時刻」まで    │        └──────────────┬───────────────┘
│ ひたすら積分する            │                       │ 最古のサンプルを取り出す
│ （カルマン更新はしない）      │                       ▼
│                           │        ┌──────────────────────────────┐
│                           │        │ EKF コア（融合時刻＝過去）        │
│      引き戻される  ◄───────┼────────┤ EKF2_PREDICT_US 周期（既定 100 Hz）│
│  correctOutputStates()    │        │                              │
│                           │        │ 予測 + カルマン更新             │
│                           │        │ GNSS/気圧/磁気/フロー/外部Vision │
└─────────┬─────────────────┘        │ を「観測が行われた過去の時刻」で融合│
          │                          └──────────────────────────────┘
          ▼ ★ publish されるのはこちら
  vehicle_attitude / vehicle_local_position / vehicle_global_position
```

`EstimatorInterface::setIMUData()`（`src/modules/ekf2/EKF/estimator_interface.cpp`）が
この分岐そのものです:

```cpp
// the output observer always runs        ← 毎 IMU サンプル
_output_predictor.calculateOutputStates(imu_sample.time_us, imu_sample.delta_ang,
                                        imu_sample.delta_ang_dt,
                                        imu_sample.delta_vel, imu_sample.delta_vel_dt);

// ダウンサンプルが溜まったときだけバッファに積む
if (_imu_down_sampler.update(imu_sample)) {
    _imu_buffer.push(imu_downsampled);
    // 融合時刻 = バッファの「最古」= 過去の時刻
    _time_delayed_us = _imu_buffer.get_oldest().time_us;
}
```

#### なぜ 2 段に分けるのか

**GNSS は 100〜200 ms 遅れて届きます。** それを「今の観測」として融合すると、
過去の位置を現在の位置だと思い込んで推定が歪みます。

そこで:

1. **EKF コアは過去（融合時刻）で融合する** — 観測が実際に行われた時刻に合わせるので、
   遅延による歪みが出ない。ただし出てくる推定値は 200 ms 前のもの
2. **出力予測器が過去から現在まで IMU だけで外挿する** — カルマン更新はせず積分だけ。
   遅延ゼロで現在時刻の値が得られる

**制御が低遅延の姿勢・位置を使えるのはこの構造のおかげです。**

#### 2 つの整合を取る仕組み

出力予測器は積分しかしないので、放っておくとコアの推定からずれていきます。
`OutputPredictor::correctOutputStates()` が **相補フィルタ**で引き戻します:

```cpp
// コアの状態と、出力予測器の「同じ過去時刻での値」を比較する
const Vector3f vel_err(vel_state - output_delayed.vel);
const Vector3f pos_err(pos_state - output_delayed.pos);

_output_tracking_error(1) = vel_err.norm();
_output_tracking_error(2) = pos_err.norm();

// 比例 + 積分ゲインで補正量を作り、バッファ全体に適用する
_vel_err_integ += vel_err;
const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

_pos_err_integ += pos_err;
const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;
```

追従できているかは `estimator_status.output_tracking_error`
（角度 [rad] / 速度 [m/s] / 位置 [m] の 3 要素）で確認できます。

| パラメータ | 既定値 | 意味 |
| --- | --- | --- |
| `EKF2_PREDICT_US` | 10000 µs（= 100 Hz） | **EKF コアの融合周期** |
| `EKF2_DELAY_MAX` | 200 ms | **遅延バッファ長 = 許容する最大センサ遅延** |
| `EKF2_IMU_CTRL` | — | IMU のどの補正を有効にするか |

> [!TIP]
> **`EKF2_DELAY_MAX` を必要以上に大きくしないでください。**
> バッファが長いほど RAM を食い、コアの推定が古くなります。
> 逆に、使っているセンサの遅延より小さいと、
> その観測はバッファから溢れて融合されません。
>
> センサごとの遅延はそれぞれのパラメータで申告します:
> `EKF2_BARO_DELAY`（気圧）, `EKF2_MAG_DELAY`（磁気）, `EKF2_RNG_DELAY`（距離計）,
> `EKF2_OF_DELAY`（フロー）, `EKF2_EV_DELAY`（外部Vision）, `EKF2_ASP_DELAY`（対気速度）。
> **GNSS には専用の遅延パラメータがありません** — 受信機が観測時刻を
> メッセージに載せてくるため、EKF はそのタイムスタンプを直接使います。

### 3 つの出力の出自は違う

`vehicle_local_position` の位置・速度・加速度は **同じ由来ではありません。**

| 出力 | 出自 | カルマン状態か |
| --- | --- | --- |
| `x/y/z`（位置） | 状態 `pos` を出力予測器が現在時刻へ外挿 | **○ 状態** |
| `vx/vy/vz`（速度） | 状態 `vel` を同様に外挿 | **○ 状態** |
| `ax/ay/az`（加速度） | **加速度計 Δv のバイアス補正・NED 回転・重力除去・平均** | **× 状態ではない** |

加速度の出所を追うと `EKF2.cpp` → `estimator_interface.h` → `output_predictor.cpp` で、
最終的にこうなっています:

```cpp
// src/modules/ekf2/EKF/output_predictor/output_predictor.cpp
matrix::Vector3f OutputPredictor::getVelocityDerivative() const
{
    if (_delta_vel_dt > FLT_EPSILON) {
        return _delta_vel_sum / _delta_vel_dt;   // 積算した Δv を時間で割るだけ
    } else {
        return matrix::Vector3f(0.f, 0.f, 0.f);
    }
}
```

`_delta_vel_sum` は上で見た `delta_vel_earth`（バイアス補正 → NED 回転 → 重力除去済み）を
そのまま足し込んだもの。**カルマンフィルタを一度も通っていません。**

> [!IMPORTANT]
> **EKF2 は加速度を推定していません。** 24 誤差状態に加速度は含まれず、
> `ax/ay/az` は「フィルタされていない、ほぼ生の平均加速度」です。
>
> だから位置制御はこのフィールドを使わず、
> **自分でフィルタ済み速度を微分して D 項を作ります**
> （[04 章](04_control_cascade.md#_vel_dot-はどこから来るのか)）。
> 生の加速度をそのまま D 項に入れるとノイズで制御が荒れるためです。

### 補正源（aid sources）

`src/modules/ekf2/EKF/aid_sources/` の各ディレクトリが 1 つの補正源に対応します。

| ディレクトリ | 補正源 | 主なパラメータ |
| --- | --- | --- |
| `gnss/` | GNSS 位置・速度・方位 | `EKF2_GPS_CTRL` |
| `barometer/` | 気圧高度 | `EKF2_BARO_CTRL` |
| `magnetometer/` | 磁気（方位） | `EKF2_MAG_TYPE` |
| `range_finder/` | レーザ・超音波距離計 | `EKF2_RNG_CTRL` |
| `optical_flow/` | オプティカルフロー | `EKF2_OF_CTRL` |
| `external_vision/` | **外部 Vision / モーキャプ** | `EKF2_EV_CTRL` |
| `aux_global_position/` | 補助的な絶対位置 | — |
| `auxvel/` | 補助速度 | — |
| `airspeed/`, `sideslip/`, `drag/` | 固定翼向け | — |
| `ranging_beacon/` | UWB 等の測距ビーコン | — |
| `gravity/` | 重力ベクトル（低速時の姿勢補正） | — |

`ZeroVelocityUpdate` / `ZeroGyroUpdate` は「静止していると判定したときに
速度 0・角速度 0 を仮想観測として与える」テクニックで、地上でのドリフトを抑えます。

### 高度基準の選択

`EKF2_HGT_REF` で「どのセンサを高度の基準にするか」を選びます
（気圧 / GNSS / 距離計 / 外部Vision）。**屋内飛行では距離計や外部Vision、
屋外では気圧か GNSS** が典型です。

> [!WARNING]
> 高度基準の選択ミスは墜落に直結します。
> 屋内で気圧基準のままドアの開閉による気圧変動を拾い、機体が上下する事故は典型例です。

---

## 3.5 外部の推定値を PX4 に入れる ★ 自律制御で最重要

GPS の効かない屋内や、SLAM・モーションキャプチャを使う場合、
**外部で計算した位置・姿勢を EKF2 に流し込む**ことができます。
これは自律制御機を作るうえで極めて重要な経路です。

### 入力トピック

| トピック | 用途 | 型 |
| --- | --- | --- |
| `vehicle_visual_odometry` | VIO / SLAM の出力 | `VehicleOdometry` |
| `vehicle_mocap_odometry` | モーションキャプチャ | `VehicleOdometry` |

EKF2 側の購読は `src/modules/ekf2/EKF2.hpp` の `_ev_odom_sub` です。

### 送り方

**ROS 2 の場合**（`src/modules/uxrce_dds_client/dds_topics.yaml` に定義済み）:

```
/fmu/in/vehicle_visual_odometry   → px4_msgs::msg::VehicleOdometry
/fmu/in/vehicle_mocap_odometry    → px4_msgs::msg::VehicleOdometry
```

**MAVLink の場合**: `ODOMETRY` / `VISION_POSITION_ESTIMATE` /
`ATT_POS_MOCAP` メッセージ（`src/modules/mavlink/mavlink_receiver.cpp` で処理）。

### 有効化パラメータ

| パラメータ | 意味 |
| --- | --- |
| `EKF2_EV_CTRL` | どの外部Vision情報を使うか（水平位置 / 垂直位置 / 速度 / 方位のビットマスク） |
| `EKF2_HGT_REF` | 高度基準を外部Visionにする場合はここを設定 |
| `EKF2_EV_DELAY` | 外部Vision の遅延補償 [ms] |
| `EKF2_EV_POS_X/Y/Z` | センサ取り付け位置のオフセット |

> [!IMPORTANT]
> **座標系に注意してください。** `VehicleOdometry` は PX4 の慣習に従います
> （位置は NED または FRD、姿勢は NED→body のクォータニオン）。
> ROS の標準は ENU / FLU なので、**必ず変換が必要**です。
> 変換については [付録 A2](A2_frames_params.md) を参照してください。
> `pose_frame` / `velocity_frame` フィールドで座標系を明示できます。

### 遅延の影響

外部推定値は必ず遅延します。`EKF2_EV_DELAY` を正しく設定しないと、
EKF2 が「過去の観測を現在の観測として」融合してしまい、位置が振動します。
**タイムスタンプを正しく打つのが最も重要**です（PX4 の `timestamp_sample` に
観測時刻を、`timestamp` に送信時刻を入れる）。

時刻同期には `timesync_status`（uXRCE-DDS が自動で行う）を使います。

---

## 3.6 推定の健全性を確認する

自律制御では「推定が信用できるか」の判定が安全に直結します。

### `vehicle_local_position` の妥当性フラグ

`msg/versioned/VehicleLocalPosition.msg` に含まれるフラグ:

| フィールド | 意味 |
| --- | --- |
| `xy_valid` / `z_valid` | 水平／垂直位置が有効 |
| `v_xy_valid` / `v_z_valid` | 水平／垂直速度が有効 |
| `xy_global` / `z_global` | 緯度経度／標高への変換基準がある |
| `dead_reckoning` | 推測航法中（絶対位置補正なし = 徐々にずれる） |
| `eph` / `epv` | 水平／垂直位置の標準偏差 [m] |
| `evh` / `evv` | 水平／垂直速度の標準偏差 [m/s] |
| `heading_good_for_control` | 方位が制御に使える精度か |

> [!TIP]
> **自律制御プログラムは、設定値を送る前にこれらのフラグを確認すべきです。**
> 特に `xy_valid` が false のときに位置設定値を送っても制御されません。
> Offboard では PX4 側もチェックしています（[07 章](07_route_offboard.md)）。

### リセットカウンタ

`xy_reset_counter`, `z_reset_counter`, `heading_reset_counter` は
**推定値がジャンプしたとき**に増加します。同時に `delta_xy`, `delta_z`, `delta_heading` に
ジャンプ量が入ります。

外部で位置制御ループを組んでいる場合、
**このカウンタの変化を検出して自分の設定値も同じだけシフトさせる**必要があります。
そうしないと、EKF がリセットした瞬間に機体が急に動きます。

PX4 内部でも同じ処理をしています
（`mc_pos_control` の Takeoff や `flight_mode_manager` の FlightTask 内）。

### 診断トピック

```sh
listener estimator_status          # 革新量（innovation）とテスト比
listener estimator_status_flags    # どの補正源が有効かのビットフラグ
listener estimator_innovations     # 各センサの革新量
listener estimator_sensor_bias     # 推定されたバイアス
```

**革新量（innovation）** = 「観測値 − 予測値」。
これが大きいまま続くなら、センサかモデルのどちらかが間違っています。
`estimator_status` の `*_test_ratio` が 1.0 を超えるとそのセンサは棄却されます。

### 出力予測器の追従誤差

`estimator_status.output_tracking_error` は **出力予測器が EKF コアに追従できているか**を
示す 3 要素の配列です（[2 段構造](#2-段構造--ekf-コアと出力予測器は並列に走る)参照）。

| 要素 | 内容 | 単位 |
| --- | --- | --- |
| `[0]` | 姿勢の追従誤差 | rad |
| `[1]` | 速度の追従誤差 | m/s |
| `[2]` | 位置の追従誤差 | m |

平常時は小さい値で安定します。**継続的に大きい場合**は、
コアの推定と IMU 積分が食い違っている — つまり
IMU のバイアスやスケール誤差、あるいは `EKF2_PREDICT_US` と IMU レートの
不整合を疑ってください。

```sh
listener estimator_status
```

---

## 3.7 代替の推定器

| モジュール | 用途 |
| --- | --- |
| `ekf2` | **標準**。ほぼ常にこれを使う |
| `attitude_estimator_q` | 姿勢のみの軽量推定（`ATT_EN=1`）。位置が不要な機体向け |
| `local_position_estimator` | 旧世代の位置推定（`LPE_EN=1`）。非推奨 |

起動時の分岐は `ROMFS/px4fmu_common/init.d/rcS` にあります
（`EKF2_EN` / `ATT_EN` / `LPE_EN` で切り替え）。

外部で INS を持っている場合は `external_ins_local_position` /
`external_ins_attitude` というトピック別名も用意されています
（`msg/versioned/VehicleLocalPosition.msg` の `# TOPICS` 行を参照）。

---

## 3.8 この章のまとめ

- **生センサ → `sensors` モジュール → EKF2 → 制御用トピック** の 3 段構成
- 制御が使うのは `vehicle_angular_velocity` / `vehicle_attitude` / `vehicle_local_position`
- `vehicle_angular_velocity` のフィルタ設定が制御品質を大きく左右する
- EKF2 は姿勢・速度・位置に加え **センサバイアスと風速も推定**する
- 遅延補償と出力予測器のおかげで、低遅延の姿勢・位置が得られる
- **外部推定値は `vehicle_visual_odometry` / `vehicle_mocap_odometry` から注入**（`EKF2_EV_CTRL`）
- 自律制御では `*_valid` フラグと **リセットカウンタ**を必ず監視すること

次章はいよいよ制御の本体です。

---

[← 前へ: 02. uORB と実行モデル](02_uorb_runtime.md) | [目次](README.md) | [次へ: 04. 制御カスケード →](04_control_cascade.md)
