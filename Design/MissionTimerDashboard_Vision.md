# Mission Timer & Scoreboard – 完成像と開発方針（オペレーション・ダッシュボード）

最終更新: 2025-09-07

本書は「Mission Timer & Scoreboard」を競技中の意思決定を支える1画面ダッシュボードへ拡張するためのビジョン、設計指針、段階的ロードマップを示します。MVP仕様は `Design/MissionTimerScoreboard.md` を参照し、本書はその上位計画（完成像）です。

## ビジョン（1画面で判断可能に）
- 目的: 競技中の時間・得点・リスク（未帰還/通信/負荷/進捗）を1画面で可視化し、撤収・継続・リトライの意思決定を即断できること。
- 前提: オフライン動作を第一（バックエンド無しでも稼働）。ROS2連携はRead-onlyで任意（安全に追加）。
- 配布: ローカルWeb（将来Tauri/単体バイナリ）。HTTPS環境ではローカルWSSで安全に接続。

## 画面レイアウト（完成形）
- 上段（固定）
  - タイマー＆主要操作（Start/Pause/Reset/Retry/Settings）
  - ライブスコア（Current / If Finish Now / Return Penalty 状態）
  - グローバル警告バー（残時間、未帰還、通信、負荷、設定の注意を集約）
- 中段（2列グリッドのウィジェット）
  - Tasks表（係数/減点/ON-OFF、合計）
  - Robots（全機戻りポリシー、戻りトグル、カットオフ推定）
  - QR Tracker（検出一覧・件数・重複統合）
  - Network Health（RTT/ロス/ジッタの信号灯とスパークライン）
  - Sim Load（calculated/simulation time 比のゲージ）
  - Event Log（操作/ROSイベント/警告の時系列）
- 下段（補助）
  - Mission Factors（Ca手動/Cm/Dc）
  - Profiles（予選/準決/決勝プリセットの保存/読込）
  - Export（CSV/JSON）
- 表示モード
  - Compact（小型ゲージで要点のみ）/ Focus（特定ウィジェット拡大）切替

## 主要ウィジェット（要件）
- タイマー/操作
  - プリセット（15/20/25/Custom）、Space/R/N/+/− 等ショートカット
  - ペース指標（予定比/余裕秒）
- スコアボード
  - Current（Piなし、Dr適用後）/ If Finish Now（Pi込み、Dr適用後）
  - 係数適用状態（Ca/Cm/Dc）と丸め表示、内訳ツールチップ（Σ→Ca→Cm→Pi→Dc→Dr）
- Tasks
  - 行追加/複製、ON/OFF、カテゴリ、Type k/Detail k/Ce/Cr/Dt、メモ
  - 代表タスクのテンプレ、列ヘッダから一括適用、Retry時にスナップショット保存
- Robots
  - 追加/削除、戻り状態、判定モード（All/Any、既定はAll）
  - Return Planner（帰還必要時刻の推定: 距離/速度/安全マージン手入力→段階アラート）
- QR Tracker
  - 値/時刻/件数、重複統合、簡易タグ（ロボID/ミッション）、手動入力、CSV/JSON出力
- Network Health
  - RTT/ロス/ジッタの信号灯（緑/黄/赤）と直近スパークライン。閾値超過は警告バーへ昇格
- Sim Load
  - calculated/simulation time 比（< 2.0 を遵守）。履歴スパークライン
- Event Log
  - 操作（Start/Pause/Retry/設定変更）とROS2イベント（robot_returned/qr）を記録・フィルタ
- Profiles/Export
  - プロファイル保存/読込（時間/丸め/Ca/Cm/Dr/表示桁/既定係数）
  - Export: CSV（Tasks/QR/イベント）/ JSON（完全状態、復元用）

## 仕様上の既定（継承）
- Return Penalty（Dr）: 全機戻りを満たさない場合、ミッション得点に 0.5×（方式1）。既定ポリシーはAll-return（切替可能）
- Time Bonus（Pi）: 残秒× 2/60、丸めは四捨五入、表示2桁（設定で変更可）
- Autonomy Factor（Ca）: 手動適用（チェックONで係数反映）
- Map Factor（Cm）: 既定1.0（無効）
- 内部計算は倍精度、表示は設定桁数

## データモデル（概略）
- Mission: `{ durationSec, startedAt, paused, elapsedOffsetMs, factors{ caApply, caValue, cmValue, dcValue }, rounding{ pi, display }, drPolicy: 'all'|'any', history[] }`
- Task: `{ id, enabled, category, typeK, detailK, Ce, Cr, Dt, note }`
- Robot: `{ id, name, returned, cutoffSec? }`
- QR: `{ id, value, timestamp, robot?, missionId? }`
- NetStats: `{ rttMs, lossPct, jitterMs, updatedAt }`
- SimLoad: `{ ratio, window[], updatedAt }`
- Event: `{ ts, type, payload }`
- Profile: `{ id, name, missionDefaults, rounding, factors, drPolicy, displayPrecision, defaultCoeffs }`

## 連携/通信（任意）
- Backend: FastAPI + WebSocket `/ws`、REST `/api/state`（スナップショット）
- イベント: `snapshot / robot_returned / qr / net / simload / ping`
- ROS2購読（設定化）: ロボ戻り `std_msgs/Bool`、QR `std_msgs/String`。将来、ネット/負荷は代理トピック or サイドカー測定
- HTTPS/WSS: `--ssl-certfile`/`--ssl-keyfile` でTLS起動。フロントは `https:` 時に `wss://localhost:8443/ws` を既定とする

## 信頼性/運用
- オフラインファースト（バックエンド無しで80%の機能）
- 永続化: localStorage + 明示エクスポート（バックアップ/持ち運び）
- パフォーマンス: 60fps維持（仮想リスト、計算デバウンス、必要に応じWeb Worker）
- アクセシビリティ: カラーブラインド配慮（色に依存しない表示/アイコン併用）/ フルスクリーン/キオスク
- 安全操作: Reset/Retry/大量削除/上書きは確認＋短時間Undo

## テスト方針
- 単体: スコア計算（Pi丸め、Ca/Cm/Dr適用順）、時間境界（残0/1/最大）、プロファイル適用
- E2E: タイマー±1秒、ショートカット、Undo/Confirm、WSS接続、Drポリシー切替の反映
- 回帰: 主要プロファイルでUI数値のスナップショット比較
- 手動: 通信劣化（遅延/ロス/ジッタ）と負荷>2.0での警告挙動

## ロードマップ（段階実装）
- フェーズA（中核機能）
  - スコア内訳のドリルダウン、Profiles、Export、Retry履歴ビュー
  - Return Planner（簡易: 速度/距離/マージン手入力→カットオフ推定）
  - WSS対応（TLSフラグ追加＋フロント既定URLの自動切替）
- フェーズB（運用強化）
  - QR Tracker（重複統合＋CSV/JSON）、Event Log、Undo/Confirm
  - Network Health/Sim Load の簡易ゲージ＋警告バー連携
- フェーズC（高度化）
  - Return Planner高度化（距離推定の外部入力/ROS2購読）、表示カスタム（Compact/Focus/列表示切替）
  - パッケージング（Tauri or 単体バイナリ）
- フェーズD（任意連携）
  - ROS2追加イベント（位置/姿勢→自動帰還判定）、リンク統計サイドカー

## 受け入れ条件（完成版）
- 1画面で「時間/現在点/今ゴール点/未帰還/通信/負荷/QR/イベント」が視認可能
- Drポリシー（All/Any）切替がスコアと警告に即時反映
- プロファイル保存/読込とCSV/JSONエクスポートが機能
- HTTPS環境でもWSS接続が可能（混在コンテンツ回避）
- タイマー誤差±1秒以内、丸め・係数適用順が仕様通り

---

用語対応（UI/Docs）: Pi→Time Bonus、Ca→Autonomy Factor、Cm→Map Factor、Dc→Manual Deduction、Dr→Return Penalty。

参考: MVP仕様は `Design/MissionTimerScoreboard.md`、現状の実装は `apps/mission-timer/frontend/*` および `apps/mission-timer/backend/*` を参照。

