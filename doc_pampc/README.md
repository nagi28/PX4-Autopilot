# doc_pampc — PAMPC を PX4 に載せるためのプロジェクト文書

PAMPC（Perception-Aware Model Predictive Control, Falanga et al., IROS 2018）を
コンパニオンコンピュータで走らせ、PX4 に接続する**この案件固有**の検討をまとめたものです。

| ファイル | 内容 |
| --- | --- |
| [`rate_mismatch.md`](rate_mismatch.md) | PAMPC の指令レートと PX4 の角速度ループレートは整合するのか。論文の 0.1 s が離散化ステップであって制御周期ではないこと、RTI スキーム、本当の制約は姿勢ループの帯域であること |
| [`integration_guide.md`](integration_guide.md) | 実装手順。送るトピック、publish 順序、座標系・符号・単位の変換、パラメータ設定、遅延予算、失敗モード、段階的な立ち上げ |

---

## このディレクトリと `docs_ja/` の役割分担

リポジトリ内に日本語ドキュメントが 2 つあります。**役割で分けています。**

| ディレクトリ | 役割 | 想定読者 |
| --- | --- | --- |
| **[`docs_ja/`](../docs_ja/README.md)** | **PX4 そのものの全体解説**（14 章）。アーキテクチャ、uORB、状態推定、制御カスケード、モード管理、自律飛行スタック、実装 3 ルート、SITL ワークフロー | PX4 を理解したい人全般 |
| **`doc_pampc/`（ここ）** | **PAMPC 案件に固有の検討**。この制御器をこの機体に載せるとき何が問題になるか | PAMPC 統合に関わる人 |

**判断基準:** PX4 の仕様・実装の説明は `docs_ja/`、
「PAMPC だとどうなるか」は `doc_pampc/`。

PX4 側の前提知識でここに書いていないものは、`docs_ja/` を参照してください。
特に関連が深いのは:

- [`docs_ja/04_control_cascade.md`](../docs_ja/04_control_cascade.md) —
  4 段カスケード、**角速度設定値のラッチと陳腐化チェックの不在**、どの階層に注入できるか
- [`docs_ja/07_route_offboard.md`](../docs_ja/07_route_offboard.md) —
  Offboard の要件（`OffboardControlMode` のフラグ優先順位、`COM_OF_LOSS_T`）、NED/ENU 変換
- [`docs_ja/10_route_comparison.md`](../docs_ja/10_route_comparison.md) —
  Offboard / PX4 内部実装 / px4_ros2 外部モードの比較と選定
- [`docs_ja/03_sensing_estimation.md`](../docs_ja/03_sensing_estimation.md) —
  EKF2 の 2 段構造と遅延補償（外部推定値を入れる場合）
