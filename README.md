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

## ローカル環境の準備（依存関係）
- 必須（バックエンド実行用）
  - Python 3.9+（推奨 3.10+）
  - パッケージ: `fastapi`, `uvicorn`, `pyyaml`
    - 例: 仮想環境内で `python -m venv .venv && source .venv/bin/activate`（Windowsは `.venv\\Scripts\\activate`）
    - 依存導入: `pip install fastapi uvicorn pyyaml`
- 任意（ROS 2 連携時）
  - ROS 2 ディストリビューション（例: Humble/Foxy 等）と `rclpy`
  - 実行前に `source /opt/ros/<distro>/setup.bash` などで環境を有効化
- 任意（GitHub Pages からの接続でTLS/WSSを使う場合）
  - mkcert（ローカル開発用証明書）
    - macOS: `brew install mkcert`
    - Windows: `choco install mkcert`
    - Linux: ディストリビューションのパッケージ or mkcert の配布手順に従う
  - セットアップ: `mkcert -install && mkcert localhost 127.0.0.1 ::1`
  - 生成された `localhost*.pem` と `*-key.pem` を `run_bridge_wss.py` に指定

起動用スクリプト（WSS）
- 追加済み: `tools/run_bridge_wss.py`
  - ドライラン（設定確認のみ）: `python tools/run_bridge_wss.py --mock --dry-run`
  - WSS起動（mkcertで発行した証明書を指定）:
    - `python tools/run_bridge_wss.py --mock --certfile ./localhost+2.pem --keyfile ./localhost+2-key.pem`
  - ROS 2 連携: `python tools/run_bridge_wss.py --config apps/mission-timer/backend/config.yaml --certfile ... --keyfile ...`

シェルスクリプト（GitHub Pages 用簡易起動）
- 追加済み: `tools/start_pages_wss.sh`
  - 既定: モックでTLS起動（証明書が見つからない場合はWSフォールバック）
    - `tools/start_pages_wss.sh`
  - ドライラン（依存未導入でもOK）
    - `tools/start_pages_wss.sh --dry-run`
  - 証明書の指定（mkcert生成物を利用）
    - `CERT=./localhost+2.pem KEY=./localhost+2-key.pem tools/start_pages_wss.sh`
  - ROS 2 設定を使う（`apps/mission-timer/backend/config.yaml`が存在する場合）
    - `USE_MOCK=0 tools/start_pages_wss.sh`
  - ポートやホストの変更
    - `PORT=9443 HOST=127.0.0.1 tools/start_pages_wss.sh`

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
