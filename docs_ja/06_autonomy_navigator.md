# 06. 自律飛行スタック — navigator と FlightTask

[← 前へ: 05. モード管理と安全機構](05_modes_safety.md) | [目次](README.md) | [次へ: 07. ルート A: Offboard 制御 →](07_route_offboard.md)

---

PX4 は「自律飛行」を最初から持っています。ミッション飛行、RTL、自動離着陸、追従、円軌道 —
これらがどう実装されているかを理解すると、
**自分の自律機能をどこに足すべきかが見えてきます**。

---

## 6.1 自律飛行の 2 層構造

```
┌───────────────────────────────────────────────────────────┐
│ navigator                    src/modules/navigator/        │
│  「どのウェイポイントへ向かうか」という 航法レベルの判断        │
│   ・ミッション（ウェイポイント列）の管理                       │
│   ・RTL の経路計画                                          │
│   ・離着陸、精密着陸                                         │
│   ・ジオフェンス監視                                         │
│  数 Hz で動く                                               │
└───────────────────────────────────────────────────────────┘
                    │
         position_setpoint_triplet
        （previous / current / next の 3 点。緯度経度高度）
                    │
┌───────────────────▼───────────────────────────────────────┐
│ flight_mode_manager          src/modules/flight_mode_manager/│
│  「今この瞬間どこにいるべきか」という 軌道レベルの生成          │
│   ・FlightTask がモードごとに設定値を作る                     │
│   ・躍度制限つき平滑化、速度制限、障害物回避                    │
│  50 Hz で動く                                               │
└───────────────────────────────────────────────────────────┘
                    │
            trajectory_setpoint
                    │
                    ▼
              mc_pos_control（04 章）
```

**役割分担が明確です:**
- `navigator` = **戦略**（どのウェイポイントか）— 緯度経度で考える
- `flight_mode_manager` = **戦術**（今どこにいるべきか）— ローカル NED で考える

---

## 6.2 navigator

`src/modules/navigator/` は独立タスクとして動きます。

### モード（`NavigatorMode` の派生クラス）

| ファイル | クラス | 対応する `nav_state` |
| --- | --- | --- |
| `mission.cpp` | `Mission` | `AUTO_MISSION` |
| `loiter.cpp` | `Loiter` | `AUTO_LOITER`（Hold） |
| `rtl.cpp`, `rtl_direct.cpp`, `rtl_mission_fast.cpp` | `RTL` 系 | `AUTO_RTL` |
| `takeoff.cpp` | `Takeoff` | `AUTO_TAKEOFF` |
| `land.cpp` | `Land` | `AUTO_LAND` |
| `precland.cpp` | `PrecLand` | `AUTO_PRECLAND` |
| `vtol_takeoff.cpp` | `VtolTakeoff` | `AUTO_VTOL_TAKEOFF` |
| `course.cpp` | `Course` | `GUIDED_COURSE` |

`navigator_main.cpp` の `Navigator::run()` が `vehicle_status.nav_state` を見て
該当モードの `run()` を呼びます。

### 出力: `position_setpoint_triplet`

`msg/PositionSetpointTriplet.msg`:

```
PositionSetpoint previous     # 直前のウェイポイント
PositionSetpoint current      # 今向かっているウェイポイント
PositionSetpoint next         # 次のウェイポイント（先読み用）
```

`PositionSetpoint`（`msg/PositionSetpoint.msg`）の主要フィールド:

| フィールド | 意味 |
| --- | --- |
| `valid` | この設定値が有効か |
| `type` | `POSITION` / `VELOCITY` / `LOITER` / `TAKEOFF` / `LAND` / `IDLE` |
| `lat`, `lon`, `alt` | **緯度経度高度（WGS84）** |
| `yaw` | 方位（NaN なら FlightTask に任せる） |
| `loiter_radius`, `loiter_pattern` | 旋回半径・パターン（円 / 8 の字） |
| `acceptance_radius` | 「到達した」と判定する半径 |
| `cruising_speed` | 希望巡航速度 |

> **`previous` / `next` があるのはなぜか:**
> 直線経路の追従には「前のウェイポイントから現在のウェイポイントへの線分」が必要で、
> 次のウェイポイントが分かればコーナーを滑らかに曲がれるからです。

### ミッションの保存

ミッション（ウェイポイント列）は `dataman` モジュールが永続化します。
GCS から MAVLink のミッションプロトコルでアップロードされ、
`mission.cpp` が読み出して実行します。

関連ファイル:
- `mission_base.cpp` / `mission_block.cpp` — ミッション実行の共通ロジック
- `mission_feasibility_checker.cpp` — アップロード時の妥当性検査
- `mission_route_provider.cpp` / `mission_route_cache.cpp` — 経路の先読み

### ジオフェンス

`geofence.cpp` が機体位置を監視し、境界を越えると `geofence_result` を発行します。
`GF_ACTION` に従って commander がフェイルセーフを発動します。

`RTLGeofenceAvoidanceHelper`（`rtl_geofence_avoidance_helper.cpp`）と
`src/lib/dijkstra` により、RTL 経路がジオフェンスを避けるよう計画されます。

---

## 6.3 flight_mode_manager と FlightTask

`src/modules/flight_mode_manager/` は、**モードごとに異なる「設定値の作り方」**を
`FlightTask` というクラス群にカプセル化しています。

### FlightTask 一覧

`src/modules/flight_mode_manager/CMakeLists.txt` の `flight_tasks_all` が登録リストです。

| タスク | 用途 |
| --- | --- |
| `Auto` | **自律モード全般**（`navigator` の出力を受けて軌道生成） |
| `ManualPosition` | Position モード（スティックで位置指令） |
| `ManualAltitude` / `ManualAltitudeSmoothVel` | Altitude モード |
| `ManualAcceleration` / `ManualAccelerationSlow` | 加速度指令方式の手動位置制御 |
| `Descend` | 降下のみ |
| `Failsafe` | フェイルセーフ時の安全な設定値 |
| `Transition` | VTOL 遷移中 |
| `AltitudeCruise` | 高度 + 巡航 |
| `Orbit` | 円軌道飛行 |
| `AutoFollowTarget` | 対象追従 |

`Orbit` と `AutoFollowTarget` は
フラッシュ容量が厳しいボード（`px4_constrained_flash_build`）では除外されます。

### FlightTask の基底クラス

`src/modules/flight_mode_manager/tasks/FlightTask/FlightTask.hpp`:

```cpp
class FlightTask : public ModuleParams
{
public:
    // モード開始時に呼ばれる。前のタスクの設定値を受け取り、継ぎ目なく引き継ぐ
    virtual bool activate(const trajectory_setpoint_s &last_setpoint);

    // モード再開時（同じタスクへの再突入）
    virtual void reActivate();

    // 毎周期の前処理（状態の取り込み）
    virtual bool updateInitialize();

    // ★ 本体。ここで設定値を計算する
    virtual bool update();

    // 計算結果を trajectory_setpoint として取り出す
    const trajectory_setpoint_s getTrajectorySetpoint();

protected:
    matrix::Vector3f _position_setpoint;
    matrix::Vector3f _velocity_setpoint;
    matrix::Vector3f _acceleration_setpoint;
    float _yaw_setpoint{};
    float _yawspeed_setpoint{};
};
```

**`update()` の中で `_position_setpoint` などのメンバに値を書けば、
それがそのまま `trajectory_setpoint` になります。**
制御しない軸には `NaN` を入れます（[04 章の NaN イディオム](04_control_cascade.md#nan-イディオム)）。

### タスクの切り替え

`FlightModeManager::start_flight_task()`
（`src/modules/flight_mode_manager/FlightModeManager.cpp`）が
`vehicle_status.nav_state` と `vehicle_control_mode` を見てタスクを選びます。

```cpp
// 自律モードなら Auto タスク
if (_vehicle_control_mode_sub.get().flag_control_auto_enabled && !nav_state_descend) {
    found_some_task = true;
    if (switchTask(FlightTaskIndex::Auto) != FlightTaskError::NoError) {
        matching_task_running = false;
        task_failure = true;
    }
}
```

> [!NOTE]
> **注目すべき設計:** タスク切り替えに失敗すると、
> 段階的に「より要求の少ないモード」へフォールバックします
> （Position → Altitude → Stabilized → Failsafe）。
> `task_failure` フラグが次の `if` の条件に混ぜられているのがその実装です。

また、**Offboard モードと外部モード（EXTERNAL1〜8）では FlightTask を走らせません**:

```cpp
if ((_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)
    || ((_vehicle_status_sub.get().nav_state >= vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
        && (_vehicle_status_sub.get().nav_state <= vehicle_status_s::NAVIGATION_STATE_EXTERNAL8))) {
    switchTask(FlightTaskIndex::None);
    return;
}
```

**これが「外部から `trajectory_setpoint` を送れる」理由です。**
FlightTask が止まるので、トピックの publisher が競合しません。

### タスクの自動登録（コード生成）

FlightTask はテンプレートからコード生成されます:

```
CMakeLists.txt の flight_tasks_all
        │
        ▼
generate_flight_tasks.py + Templates/FlightTasks_generated.{hpp,cpp}.em
        │
        ▼
FlightTaskIndex 列挙型 と _initTask() の switch 文が自動生成される
```

**新しい FlightTask を足すときは `flight_tasks_all` に名前を追加するだけ**で、
列挙型もファクトリも自動で更新されます（[08 章](08_route_internal.md)で詳述）。

### FlightTaskAuto — 自律モードの中身

`src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp` が
`position_setpoint_triplet`（緯度経度）を `trajectory_setpoint`（ローカル NED）に変換します。

主な処理:
1. 緯度経度 → ローカル NED 変換（`src/lib/geo/`）
2. `previous` → `current` の線分に沿った経路追従
3. **躍度制限つきの速度平滑化**（`src/lib/motion_planning/`）
4. ウェイポイント到達判定（`acceptance_radius`）
5. コーナーでの減速
6. yaw の生成（`MPC_YAW_MODE`: 進行方向を向く / ホームを向く / 固定 など）
7. 障害物回避（`src/lib/collision_prevention/`）

関連パラメータ: `MPC_XY_CRUISE`, `MPC_JERK_AUTO`, `MPC_ACC_HOR`,
`MPC_YAW_MODE`, `MPC_YAWRAUTO_MAX`, `NAV_ACC_RAD`, `NAV_MC_ALT_RAD`

---

## 6.4 goto_setpoint — 簡易な高レベル入口

`msg/versioned/GotoSetpoint.msg` は「ここへ行け」という最も簡単な指令です。

```
float32[3] position                    # NED [m]
bool  flag_control_heading             # 方位を制御するか
float32 heading                        # [rad]
bool  flag_set_max_horizontal_speed    # 速度上限を指定するか
float32 max_horizontal_speed           # [m/s]
bool  flag_set_max_vertical_speed
float32 max_vertical_speed
bool  flag_set_max_heading_rate
float32 max_heading_rate
```

処理するのは `src/modules/mc_pos_control/GotoControl/GotoControl.cpp` です。
**内部で平滑化してくれるので、送る側は運動学的に無理のない値である必要がありません。**

```cpp
// MulticopterPositionControl::Run() 内
const bool goto_setpoint_enable = _vehicle_control_mode.flag_multicopter_position_control_enabled
                                  && !_trajectory_setpoint_sub.updated();

if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample, goto_setpoint_enable)) {
    _goto_control.update(dt, states.position, states.velocity, states.acceleration, states.yaw);
}
```

> [!IMPORTANT]
> **`trajectory_setpoint` が来ていると `goto_setpoint` は無効になります。**
> 両方を同時に送ってはいけません。

| | `goto_setpoint` | `trajectory_setpoint` |
| --- | --- | --- |
| 送る側の責任 | 目的地だけ | **運動学的に一貫した軌道** |
| 平滑化 | PX4 がやる | **送る側がやる** |
| 送信レート | 低くてよい | 高いほど良い（滑らかさのため） |
| 向いている用途 | ウェイポイント巡航 | 軌道最適化、高速飛行、追従 |

**自律制御の第一歩としては `goto_setpoint` が最も安全で簡単です。**

---

## 6.5 その他の自律機能

| 機能 | 実装 | 説明 |
| --- | --- | --- |
| 精密着陸 | `src/modules/navigator/precland.cpp` + `landing_target_estimator` | マーカーを見て正確に着陸 |
| 追従飛行 | `tasks/AutoFollowTarget/` | GPS 位置を送ってくる対象を追う |
| 円軌道 | `tasks/Orbit/` | 指定点の周りを回る |
| 衝突回避 | `src/lib/collision_prevention/` | 距離センサ / `obstacle_distance` で減速 |
| 検知回避 (DAA) | `src/modules/navigator/DetectAndAvoid/` | 他機との衝突回避 |
| 視覚ターゲット推定 | `src/modules/vision_target_estimator/` | 着陸マーカー等の推定 |
| ペイロード投下 | `src/modules/payload_deliverer/` | グリッパ制御 |

これらは「PX4 内部に自律機能を実装する」実例集としても有用です。
似た機能を作りたければ、まずこれらを読んでください。

---

## 6.6 自作の自律機能をどこに足すか

この章の内容を踏まえた選択肢:

| やりたいこと | 足す場所 | 章 |
| --- | --- | --- |
| 新しいウェイポイント戦略（探索パターンなど） | `navigator` に `NavigatorMode` 派生を追加 | [08](08_route_internal.md) |
| 新しい軌道生成方式 | `FlightTask` 派生を追加 | [08](08_route_internal.md) |
| 独自の制御則 | 制御モジュールを差し替え / 追加 | [08](08_route_internal.md) |
| 外部の重い計算（SLAM、経路計画、学習モデル） | コンパニオンから注入 | [07](07_route_offboard.md), [09](09_route_external_mode.md) |

> [!TIP]
> **判断基準:**
> - **リアルタイム性が要る & 計算が軽い** → PX4 内部（FlightTask / モジュール）
> - **計算が重い、ライブラリが必要、開発サイクルを速くしたい** → コンパニオン
>
> 経路計画や物体認識は間違いなくコンパニオン向きです。
> 一方、姿勢制御則の改造は PX4 内部でないと成立しません。

---

## 6.7 この章のまとめ

- 自律飛行は **navigator（戦略・緯度経度・数 Hz）** と
  **flight_mode_manager（戦術・ローカル NED・50 Hz）** の 2 層
- `navigator` は `position_setpoint_triplet`（前・現・次の 3 点）を出す
- `flight_mode_manager` は **FlightTask** でモードごとの軌道生成を切り替える
- FlightTask は `flight_tasks_all` に名前を書くだけで自動登録される
- **Offboard モードと外部モードでは FlightTask が止まる**ので、
  外部から `trajectory_setpoint` を publish できる
- `goto_setpoint` は PX4 側で平滑化してくれる最も簡単な入口
- 重い計算はコンパニオン、リアルタイム制御は PX4 内部、が基本方針

次章から、実際に自分の自律制御を実装する 3 つのルートを見ていきます。

---

[← 前へ: 05. モード管理と安全機構](05_modes_safety.md) | [目次](README.md) | [次へ: 07. ルート A: Offboard 制御 →](07_route_offboard.md)
