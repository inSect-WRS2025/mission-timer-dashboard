# Mission Timer & Scoreboard（運用ダッシュボード）

WRS2025 シミュレーション災害チャレンジの競技中に、時間管理と得点見通しを1画面で確認するためのオフラインファーストなダッシュボードです。GitHub Pages で配信する静的フロントエンドと、ローカルPC上で任意起動する FastAPI バックエンド（ROS2購読・Read-only）の組み合わせで動作します。

- フロント（静的SPA）: `docs/`（GitHub Pages 配信用）
- バックエンド（任意・ROS2 Bridge）: `apps/mission-timer/backend/`
- 設計ドキュメント: `Design/`

## すぐ使う（3つのパターン）

1) 完全オフライン（バックエンドなし）
- ブラウザで `docs/index.html` を開くだけで利用できます（Chromium系推奨）。
- タイマー、タスク編集、係数設定、ロボット戻り状態、得点計算はフロント内で完結します。

2) ローカルでモック接続（WS, 平文）
- バックエンドをモックで起動し、フロントから `ws://localhost:8000/ws` に接続します。
  ```bash
  # リポジトリルートで
  python apps/mission-timer/backend/app.py --mock --host 127.0.0.1 --port 8000
  ```
- フロント画面の「ROS2 Bridge」欄の URL を `ws://localhost:8000/ws` に設定して Connect。

3) GitHub Pages（HTTPS）＋ ローカルWSS接続（推奨）
- PagesはHTTPS配信のため、`ws://` は混在コンテンツとしてブロックされます。ローカルバックエンドをTLSで起動し、`wss://localhost:8443/ws` に接続してください。
  - ローカル証明書の作成（mkcert）
    ```bash
    mkcert -install
    mkcert localhost 127.0.0.1 ::1
    # 例: localhost+2.pem, localhost+2-key.pem が生成される
    ```
  - バックエンド起動（TLS, 例: 8443）
    ```bash
    python apps/mission-timer/backend/app.py --mock \
      --host 127.0.0.1 --port 8443 \
      --ssl-certfile ./localhost+2.pem \
      --ssl-keyfile ./localhost+2-key.pem
    ```
  - フロント（Pages）側は HTTPS で読み込まれるため、既定で `wss://localhost:8443/ws` が入力されます（手動変更も可）。

4) ROS2 ありで接続（任意）
- 事前にROS2環境をsourceしてください（例: `source /opt/ros/humble/setup.bash`）。
- 設定ファイルを作成（例をコピー）:
  ```bash
  cp apps/mission-timer/backend/config.example.yaml apps/mission-timer/backend/config.yaml
  # returnedトピックやQRトピックを編集
  ```
- バックエンド起動（平文WSの例）
  ```bash
  python apps/mission-timer/backend/app.py \
    --config apps/mission-timer/backend/config.yaml \
    --host 127.0.0.1 --port 8000
  ```
- GitHub Pages 経由で使う場合は上記「3)」の手順でTLS起動（`--ssl-certfile/--ssl-keyfile`）し、`wss://` で接続してください。

## GitHub Pages の設定
- リポジトリ Settings → Pages → "Deploy from a branch" → Branch: `main` / Folder: `/docs`
- Enforce HTTPS: ON（証明書発行に数分かかることがあります）
- 付帯ファイル: `docs/.nojekyll`（同梱済み）, `docs/404.html`（同梱済み）

## 操作の基本（ショートカット）
- Space: Start / Pause
- R: Retry（スコアを0にリセットし履歴スナップショット保存）
- N: 新規タスク追加

## 既定の計算・表示
- Return Penalty（Dr）: 「全機戻り」を満たさない場合、ミッション得点に 0.5 倍（方式1）
- Time Bonus（Pi）: 残秒 × 2/60、丸めは四捨五入、表示は小数2桁（設定で変更可）
- Autonomy Factor（Ca）: 手動適用（チェックON時に係数反映）
- Map Factor（Cm）: 既定 1.0（無効）
- 表示: 内部は倍精度、表示は設定桁数（既定2桁）

## ディレクトリ構成（要点）
- `docs/` … GitHub Pages 配信対象（`index.html`, `app.js`, `styles.css`, `.nojekyll`, `404.html`）
- `apps/mission-timer/frontend/` … フロントのソース
- `apps/mission-timer/backend/` … FastAPI + WebSocket（ROS2 Bridge, `--mock`あり）
- `Design/` … 仕様（MVP/完成像/ロードマップ）

## トラブルシューティング
- Pages(HTTPS)で `ws://` に接続できない
  - 仕様です。`wss://localhost:8443/ws` へ接続してください（ローカルTLS起動が必要）。
- TLS接続で警告が出る/接続できない
  - ブラウザが証明書を信頼していない可能性があります。mkcertのインストールや証明書ファイルの指定を再確認してください。
- ROS2が見つからない/ImportError
  - `rclpy`が利用可能になるよう、ROS2環境を `source` してください。`--mock` ならROS2なしでも動作します。

## 参考
- MVP仕様: `Design/MissionTimerScoreboard.md`
- ダッシュボードの完成像・計画: `Design/MissionTimerDashboard_Vision.md`
