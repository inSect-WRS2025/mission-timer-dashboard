# Mission Timer & Scoreboard（運用ダッシュボード）

WRS2025 シミュレーション災害チャレンジ向けの、時間管理と得点見通しを1画面で可視化するオフラインファーストのダッシュボードです。フロントはGitHub Pages（静的SPA）、ROS 2連携はローカルPC上のFastAPIバックエンド（任意・Read-only）で行います。

- フロント（Pages配信用）: `docs/`
- バックエンド（ROS 2 Bridge）: `apps/mission-timer/backend/`
- 設計: `Design/`

—

## 概要
- 目的: 残時間、現在得点、「今ゴール」の得点（Time Bonus込み）と、ロボット帰還状況（Return Penalty）を即時に把握し、撤収判断を支援。
- 仕様要点:
  - Dr（Return Penalty）: 全機未帰還なら 0.5×（方式1, 既定All-return）
  - Pi（Time Bonus）: 残秒 × 2/60（四捨五入、小数2桁表示）
  - Ca（Autonomy）: 手動適用、Cm（Map）: 既定1.0、Dc（Manual Deduction）対応
  - オフライン動作OK。ROS 2連携は任意、HTTPS環境（Pages）ではローカルWSSで接続

—

## ローカルPCのセットアップ（依存のインストール）
- 必須（バックエンド実行）
  - Python 3.9+（推奨 3.10+）
  - パッケージ: `fastapi`, `uvicorn[standard]`, `pyyaml`
    - 例: `python -m venv .venv && source .venv/bin/activate`（Windows: `.venv\\Scripts\\activate`）
    - インストール: `pip install fastapi 'uvicorn[standard]' pyyaml`
- 任意（ROS 2連携時）
  - ROS 2（例: Humble/Foxy）と `rclpy`
  - 実行前に `source /opt/ros/<distro>/setup.bash`
  - 確認: `python3 -c "import rclpy; print('rclpy OK')"`
- 任意（GitHub Pagesと接続する場合のTLS/WSS）
  - mkcert（ローカル開発証明書）
    - macOS: `brew install mkcert` / Windows: `choco install mkcert` / Linux: 各ディストリのパッケージ
  - 設定: `mkcert -install && mkcert localhost 127.0.0.1 ::1`
  - 生成された `localhost*.pem` と `*-key.pem` を起動スクリプトで指定

—

## バックエンドの起動（起動用スクリプト）
- 推奨: `tools/start_pages_wss.sh`（GitHub Pagesからの接続を想定したWSS起動）
  - モック（ROSなし）でWSS起動:
    - 証明書を自動検出（見つからない場合はWSフォールバック）
    - `tools/start_pages_wss.sh`
    - 動作確認: `https://127.0.0.1:8443/api/state` がJSONで開ける
  - ROS 2連携（TLS/WSS）:
    - 設定ファイルを作成
      - `cp apps/mission-timer/backend/config.example.yaml apps/mission-timer/backend/config.yaml`
      - `config.yaml` を編集（例）
        - `robots[].returned_topic`: `std_msgs/Bool` のトピック
        - `qr_topic`: `std_msgs/String` のトピック
    - 起動（ROSをsource済みのシェルで）
      - `CERT=./localhost+2.pem KEY=./localhost+2-key.pem USE_MOCK=0 tools/start_pages_wss.sh`
    - 起動ログ例（抜粋）
      - `[bridge] Mode: ROS2 (ROS_AVAILABLE=True)`
      - `[bridge/ros] node up. robot_subs=[('Robot A','/robot_a/returned'), ...], qr_topic=/mission/qr`

—

## GitHub Pages（フロントエンド）
- 公開URL: https://insect-wrs2025.github.io/mission-timer-dashboard/
- 接続手順（Pages → ローカルWSS）
  - バックエンドを上記のWSSで起動（8443/TLS）
  - 画面の「ROS2 Bridge」のURLを `wss://localhost:8443/ws`（または `wss://127.0.0.1:8443/ws`）にして「Connect」
  - 接続中は10秒ごとにpingが流れ、`last evt` が更新（正常）

—

## UIの説明（操作・画面の意味）
- タイマー（Timer）
  - プリセット（15/20/25分）とカスタム秒、Start/Pause/Reset/Retry
  - ショートカット: Space=Start/Pause, R=Retry, N=新規タスク
- スコアパネル（Score Panel）
  - Current Score（Piなし, Dr適用後）/ If Finish Now（Pi込み, Dr適用後）を大表示
  - Breakdown（Σ→Ca→Cm→Pi→Dc→Dr）を展開表示可
- ROS2 Bridge パネル（ROS2 Bridge）
  - URL欄に `ws://` または `wss://` を入力し Connect/Disconnect
  - Status/Stats: `connected`/`disconnected`、`qr`/`robot updates`/`ping`/`last evt`
- ロボット（Robots (All return policy)）
  - 追加/削除、Returnedのトグル。Dr判定（All-return既定）に反映
- タスク（Tasks）
  - 列: Enabled, Category, TypeK, DetailK, Ce, Cr, Dt, Note, Del
  - 行のON/OFF/編集で即時計算
- ミッション係数（Mission Factors）
  - Ca適用のON/OFFと値、Cm値、Dc（手動減点）
- プロファイル（Profiles）
  - 保存/読込、選択、JSONインポート/エクスポート
- リトライ履歴（Retry History）
  - Retry時のスナップショット（時刻/Tasks/Robots/Score）一覧
- エクスポート（Export）
  - Tasks CSV、History CSV、State JSON の出力/入力
- スコア内訳（Score Breakdown）
  - Breakdownの詳細（Σ→Ca→Cm→Pi→Dc→Dr の各段階の数値）

—

## ROS 2連携テスト（例）
- 購読確認（別ターミナル。どちらもROSをsource済みで）
  - `ros2 topic info /mission/qr` → Subscribers: 1
  - `ros2 topic info /robot_a/returned` → Subscribers: 1
- PublishしてUIに反映
  - `ros2 topic pub -1 /mission/qr std_msgs/String "{data: 'QR-001'}"`
  - `ros2 topic pub -1 /robot_a/returned std_msgs/Bool "{data: true}"`
  - 期待: UIの `qr` / `robot updates` が増える、Returned/Dr表示が更新

—

## トラブルシューティング
- Pages(HTTPS)で `ws://` は混在コンテンツでブロック → `wss://localhost:8443/ws` を使用
- バックエンド起動時に `Unsupported upgrade request` → `pip install 'uvicorn[standard]'` を導入
- TLSが信頼されない → `mkcert -install` 実施、`localhost*.pem` を指定
- 接続できるがイベントが来ない
  - 起動ログが `Mode: MOCK` ならROS未source/環境不整合。ROSをsourceして再起動
  - トピック名不一致（`config.yaml` と `ros2 topic pub` を合わせる）
  - ドメインID不一致（`ROS_DOMAIN_ID` を揃える）
  - UIはpingで `last evt` が更新される（正常）

—

## 付録: 用語集（Glossary）

- ミッションスコア関係（Rulesに準拠）
  - Pm: ミッション総得点
  - Pti: 各タスクの基礎点（本ツールでは 10×Type係数×Detail係数）
  - Cei: 環境係数（Env coef）
  - Cri: ロボット係数（Robot coef）
  - Ca: Autonomy Factor（自律度係数、手動適用）
  - Cm: Map Factor（地図提出係数、既定1.0）
  - Pi: Time Bonus（残時間ボーナス = 残秒×2/60、四捨五入、表示2桁）
  - Dc: Manual Deduction（審判による減点、手入力）
  - Dr: Return Penalty（帰還ペナルティ。既定: 全機未帰還なら0.5×）

- UI上の主要ラベル
  - Current Score: 現在得点（Piなし、Dr適用後）
  - If Finish Now: いまゴールした場合の得点（Pi込み、Dr適用後）
  - Score Breakdown: 得点内訳（Σ→Ca→Cm→Pi→Dc→Dr）
  - Mission Factors: ミッション係数（Ca/Cm/Dc の設定）
  - ROS2 Bridge: ブリッジ接続（`ws://`/`wss://`）
  - Robots (All return policy): ロボットの帰還状態（既定は全機帰還でDr回避）
  - Tasks: タスク一覧（係数/減点/メモ）
  - Profiles: プロファイル保存/読込（時間・丸め・係数・Drポリシー等）
  - Retry History: リトライ時のスナップショット一覧
  - Export: CSV/JSONの入出力

- タスク列の用語
  - Category: タスク分類の簡易コード（自由入力・1文字推奨）
    - チーム内記法（例）: M / R / O / W
      - M: 例) Mobility/Manipulation（移動/操作）
      - R: 例) Recon/Rescue（偵察/救助）
      - O: 例) Object（物体関連）
      - W: Firefighting（消火、例: W-f_1〜W-f_4）
    - 注: 公式カテゴリ名に合わせてチームで統一してください（本ツールは文字列をそのまま表示）
  - Type coef / Detail coef: 種類・細目の係数（自由に設定可能）
  - Env coef (Ce): 環境係数（ルール由来の加点/係数を入力）
  - Robot coef (Cr): ロボット係数
  - Task penalty (Dt): タスク単位の減点
  - Note: 補足メモ

- ROS 2 連携用語
  - returned_topic: 各ロボットの帰還判定トピック（`std_msgs/Bool`）
  - qr_topic: QR検出の通知トピック（`std_msgs/String`）
  - ROS_DOMAIN_ID: DDSドメインID。pub/bridgeで一致させる
  - RMW_IMPLEMENTATION: ミドルウェア実装。基本は一致推奨

- その他
  - All return / Any return: Drの判定モード。既定はAll（全機帰還でペナルティ回避）
  - Display decimals / Time Bonus rounding: 表示桁数とPiの丸め設定
  - Ping / last evt: ブリッジの疎通指標（10秒ごとにpingイベントが届く）
