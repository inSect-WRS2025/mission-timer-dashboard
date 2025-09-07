# Mission Timer & Scoreboard（MVP スキャフォールド）

WRS2025 シミュレーション災害チャレンジの競技中に、時間管理と得点見通しを支援するオフライン動作の単一ページ Web アプリです。

- 単一 HTML（外部依存なし）
- 英語 UI、オフライン動作、`index.html` を開くだけで利用可能
- 既定値を実装: Dr 方式1（全機戻りでない場合 0.5倍）、Pi 四捨五入、表示2桁、Ca 手動、Cm 1.0、全機戻りポリシー

## 主な機能（MVP）
- 大きなミッションタイマー（プリセット 15/20/25 分＋カスタム）
- タスク表（カテゴリ、種類/細目係数、環境係数、ロボット係数、タスク減点、ON/OFF、メモ）
- ミッション係数（自律係数 手動適用＋値、地図提出係数、審判減点（手動））
- ロボット帰還パネル（全機戻りポリシー、各ロボのトグル）
- ライブ得点表示：Current Score（Piなし）／If Finish Now（Pi込み）
- リトライ（スコアを0にリセットし履歴スナップショット保存）、Start/Pause/Reset タイマー
- 状態の永続化（localStorage）

## 使い方
1) ブラウザ（Chromium系推奨：Chrome 等）で `frontend/index.html` を開く。
2) 上部プリセット（15:00/20:00/25:00）または Custom で時間を設定し、Start で計時開始。
3) Tasks でタスクを追加し、係数（Type k, Detail k, Ce, Cr）や減点 Dt を入力すると即時に得点が反映されます。
4) Mission Factors で必要に応じて Ca（手動）や Cm、Dc を設定。
5) Robots でロボットを追加し、戻り状態を管理。未帰還があると Dr が 0.5倍で適用されます。
6) If Finish Now は「今ゴールした場合の得点（Time Bonus込み）」を表示します。Time Bonus は 残秒×2/60 点で四捨五入、表示は小数2桁です。

### （任意）ROS2 Bridge 連携
- `backend/app.py` を起動すると、WebSocket で UI と連携できます（読み取り専用）。
- 起動例（ROS2なし・モック動作）:
  - `python apps/mission-timer/backend/app.py --mock`
- 起動例（ROS2あり）:
  - ROS 2 を source 後、`python apps/mission-timer/backend/app.py --config apps/mission-timer/backend/config.yaml`
- フロント側の「ROS2 Bridge」パネルで `ws://localhost:8000/ws` に接続。
- 受信イベント例:
  - robot_returned（各ロボの戻り状態を自動反映）
  - qr（検出数カウントを更新）

### キーボードショートカット
- Space: Start / Pause（開始・一時停止）
- R: Retry（スコア0にリセット、履歴保存）
- N: New Task（タスク追加）

## 備考（既定の動作）
- Return Penalty（Dr）: 「全機戻り」を満たさない場合、ミッション得点に 0.5 倍を適用（Time Bonus および Manual Deduction 反映後に適用）。
- Time Bonus（旧Pi）: 残り時間1秒ごとに 2/60 点。丸めは四捨五入、表示は小数2桁。
- Autonomy Factor（旧Ca）: 手動適用（チェック ON の場合のみ係数を反映）。
- Map Factor（旧Cm）: 既定 1.0（無効）。
- 設定と状態はブラウザの localStorage に保存されます。

## 今後の予定（Roadmap）
- プロファイルの保存/読込、CSV/JSON エクスポート
- リトライ履歴ビュー
- パッケージング（Tauri / PyInstaller）
- 任意: Backend（FastAPI）や ROS2（Read-only）の統合

詳細な設計は `../../Design/MissionTimerScoreboard.md` を参照してください。
