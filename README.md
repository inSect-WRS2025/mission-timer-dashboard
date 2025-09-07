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
- 公開URL: https://yoshiko-kulala.github.io/mission-timer-dashboard/
- 接続手順（Pages → ローカルWSS）
  - バックエンドを上記のWSSで起動（8443/TLS）
  - 画面の「ROS2 Bridge」のURLを `wss://localhost:8443/ws`（または `wss://127.0.0.1:8443/ws`）にして「Connect」
  - 接続中は10秒ごとにpingが流れ、`last evt` が更新（正常）

—

## UIの説明（操作・画面の意味）
- タイマー
  - プリセット（15/20/25分）とカスタム秒、Start/Pause/Reset/Retry
  - ショートカット: Space=Start/Pause, R=Retry, N=新規タスク
- スコアパネル
  - Current Score（Piなし, Dr適用後）/ If Finish Now（Pi込み, Dr適用後）を大表示
  - Breakdown（Σ→Ca→Cm→Pi→Dc→Dr）を展開表示可
- ROS2 Bridge パネル
  - URL欄に `ws://` または `wss://` を入力し Connect/Disconnect
  - Status/Stats: `connected`/`disconnected`、`qr`/`robot updates`/`ping`/`last evt`
- Robots パネル
  - 追加/削除、Returnedのトグル。Dr判定（All-return既定）に反映
- Tasks テーブル
  - 列: Enabled, Category, TypeK, DetailK, Ce, Cr, Dt, Note, Del
  - 行のON/OFF/編集で即時計算
- Mission Factors
  - Ca適用のON/OFFと値、Cm値、Dc（手動減点）

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

## 参考
- MVP仕様: `Design/MissionTimerScoreboard.md`
- 完成像・計画: `Design/MissionTimerDashboard_Vision.md`
