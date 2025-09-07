# Mission Timer & Scoreboard 開発計画（仕様合意用）

最終更新: 2025-09-06

本計画は、WRS2025 シミュレーション災害チャレンジ向けに、競技中の時間管理と得点見通しを支援する「Mission Timer & Scoreboard」のMVP実装に向けた仕様・設計・工程を示します。

## 目的と範囲
- 目的: ミッション中の時間・得点・撤収（ゴール）判断材料を一画面で可視化する。
- MVP範囲: タイマー、タスク入力/編集、係数設定、現在得点と「今ゴール」得点（Pi込み）計算、ロボット帰還状態管理、リトライ管理、プロファイル保存。
- 非範囲(MVP): ROS2自動連携、運営スコア自動反映、ネットワーク共有、他ツール連携。

## 前提・用語
- 競技時間: 予選15分／準決勝・決勝は15〜25分（プリセット + カスタム）。
- 得点式: `Pm = {Σ(Pti × Cri × Cei − Dti)} × Ca × Cm + Pi − Dr − Dc`。
- Pi: 残り時間1秒ごとに 2/60 点加点。
- 係数/減点: Ca, Cm, Ce, Cr, Dt, Dr, Dc をUIで入力/選択。

## 合意済みデフォルト（2025-09-06）
- Drの解釈/既定: 方式1「全機戻らないと0.5倍」（乗算）を既定。
- Piの丸め規則: 四捨五入（表示は小数2桁）。
- Caの既定: 手動適用（チェックボックス等）を既定。
- Cmの既定値: 1.0（デフォルト無効）。
- ロボット帰還判定の既定: 「全機戻り」。
- UI言語: 英語のみ（first release）。
- 実装形態: ローカルWebアプリ（バックエンド + フロントエンド）。

## ユーザーストーリー（MVP）
- 操作者として、プリセット時間を選んで Start で計時を開始・Pause/Resetできる。
- スコア担当として、タスク（M/R/O/W）を追加し、係数/減点を入力すると総得点が即時更新される。
- 操作者として、「今ゴール」得点（Pi込み）を常時確認し、撤収判断に使える。
- スコア担当として、各ロボの帰還状態を管理し、Drリスクを即時に把握できる。
- チームとして、Retry を押すと現ミッション得点が0にリセットされ、履歴が保存される。
- チームとして、係数・丸め規則・Drポリシーをプロファイルとして保存/切替できる。

## UI設計（1画面）
- Top Bar: Remaining / Elapsed（大表示）, Start / Pause / Reset / Retry。
- Score Panel:
  - Current Score（大） / If Finish Now（Pi込み, 大） / Dr Risk（色/アイコン）。
  - Ca/Cm の適用状態（ミッション単位）。
- Task Table:
  - Columns: Category(M/R/O/W), TypeCoeff, DetailCoeff, Ce, Cr, Dt, Enabled, Note。
  - Footer: Σ(Pti×Cri×Cei−Dti)。
- Robots Panel: Robotごとの Returned トグル、判定モード（All return / Any return）。既定は All。
- Settings Modal: Mission time, rounding(Pi/Display), Ca mode(Manual), Cm value, Dr mode(方式1), display precision(2 decimals)。
- Shortcuts: Space=Start/Pause, R=Retry, N=New Task。

## データモデル（概略）
- Mission: `{ durationSec, startedAt, paused, retries[], profileId, factors{ CaMode, CmValue, DrMode, rounding{pi, display} } }`
- Task: `{ id, category, typeCoeff, detailCoeff, Ce, Cr, Dt, enabled, note }`
- Robots: `[{ id, name, returned }]`
- Profile: `{ id, name, defaultDuration, rounding, CaMode, DrMode, defaultCoeffs, displayPrecision }`
- History(retries): `[{ timestamp, reason, tasksSnapshot, robotsSnapshot }]`

## 計算仕様（MVP）
- `Pti = 10 × 種類係数 × 細目係数`。
- Σタスク: `sum( Pti × Cri × Cei − Dti )`。
- Ca: 既定は Manual（チェックONで適用, 値は入力 or プロファイル既定）。
- Cm: 入力値（1.0〜1.2）をミッション単位で適用（既定1.0）。
- Pi: `残秒 × (2/60)`、丸めは四捨五入、表示は小数2桁。
- Dr: 方式1（All return満たさない場合は 0.5× を適用）。方式2（固定減点入力）は将来オプションとして実装フックを残す。
- Dc: 固定減点（手入力, 0既定）。
- 表示: 内部は倍精度、表示は2桁（設定変更可）。

## リトライ仕様
- Retryで現ミッション得点は0に再計算（履歴へスナップショット保存）。
- タイマーは継続（競技時間は進行）。
- タスク/ロボット状態の初期化はオプション（保持/クリア）。既定は保持。

## 保存/エクスポート
- LocalStorage/JSONファイルに設定・プロフィール・履歴を保存。
- Export: CSV（タスク一覧、履歴）、JSON（完全状態）。

## 技術方針（MVP）
- 形態: ローカルWeb（Backend: Python FastAPI + Uvicorn, Frontend: React + Vite or SvelteKit）。
- 配布: v0 ソース起動、v1 でPyInstallerまたはTauriでバンドル。
- 時刻: `monotonic` ベースでドリフト補正。

## テスト計画
- 単体テスト: スコア計算（Pi丸め、Ca/Cm/Dr適用）、時間境界（残0/1/最大）。
- UIテスト: タイマー誤差（±1秒以内）、編集の即時計算、ショートカット動作。
- シナリオ: 1) 単一タスク、2) 複数タスク＋係数、3) Retry履歴、4) Dr判定切替。

## リスクと対策
- 未確定仕様（運営更新）: 設定・プロファイルで吸収、値は外部化。
- 誤操作: 重要操作に確認/Undo（短時間）。
- ブラウザ差異: ローカル環境想定で検証（Chromium系推奨）。

## 受け入れ条件（MVP）
- 15分プリセットで動作、Current/If Finish Now が瞬時に更新される。
- タスク編集で総得点が正しく再計算される。
- Dr方式1が既定で機能し、RobotsパネルのReturnedで挙動が変わる。
- Retryで得点0に戻り、履歴が保存される。
- プロファイルの保存/読込ができる。

## フェーズ計画
- 仕様確定（本書合意）: 0.5日
- フェーズ1: MVP実装（計算ロジック＋UI骨格＋保存）: 3–4日
- フェーズ2: UX改善（ショートカット・履歴ビュー・警告UI・プロファイル強化）: 2日
- フェーズ3: パッケージング（オフライン配布, ワンバイナリ）: 1–2日
- フェーズ4: 任意拡張（ROS2購読、他ツール連携）: 別途

---

変更履歴:
- 2025-09-06: 初版作成。デフォルト方針（Dr方式1, Pi四捨五入/表示2桁, Ca手動, Cm=1.0, All return, 英語UI, ローカルWeb）を合意反映。
