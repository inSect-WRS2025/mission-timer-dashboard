# Mission Timer ROS 2 Bridge (Optional)

FastAPI server that integrates with ROS 2 (rclpy) in read-only mode and streams useful events to the Mission Timer frontend.

- Subscribes to configured topics (robots' returned flags, QR detections)
- Serves a WebSocket `/ws` that emits events to the UI
- Serves REST `/api/state` for snapshots
- Read-only: does not publish or control the simulation

## Requirements
- Python 3.10+
- ROS 2 (with `rclpy`) sourced in your shell (e.g., `source /opt/ros/humble/setup.bash`)
- `fastapi`, `uvicorn`, `pyyaml`

## Install
```bash
python -m venv .venv
source .venv/bin/activate
pip install fastapi uvicorn pyyaml
# rclpy provided by ROS 2 environment; ensure it is sourced
```

## Configure
Copy and edit the example config:
```bash
cp apps/mission-timer/backend/config.example.yaml apps/mission-timer/backend/config.yaml
```
Edit `config.yaml` to map robot names to their returned topics and QR topic.

## Run
```bash
# in the repo root
source /opt/ros/<distro>/setup.bash   # e.g., humble
python apps/mission-timer/backend/app.py --config apps/mission-timer/backend/config.yaml --host 0.0.0.0 --port 8000
```
Open `apps/mission-timer/frontend/index.html` in your browser and use the ROS2 Bridge panel to connect to `ws://localhost:8000/ws`.

## Event Schema
- `snapshot`: `{ type: "snapshot", robots: [{name, returned}], qrDetections: [{value, timestamp}] }`
- `robot_returned`: `{ type: "robot_returned", name, returned }`
- `qr`: `{ type: "qr", value, timestamp }`

If ROS 2 is not available, the server can still run in mock mode using `--mock` (generates sample events).

## HTTPS/WSS with GitHub Pages (local TLS endpoint)
When the frontend is served from GitHub Pages (HTTPS), browsers may block `ws://` as mixed content. Provide a local WSS endpoint so the page can connect securely.

1) Generate a locally-trusted certificate (mkcert):
```bash
mkcert -install
mkcert localhost 127.0.0.1 ::1
# Produces files like: localhost+2.pem and localhost+2-key.pem
```

2) Run the backend with TLS (example on port 8443):
```bash
python apps/mission-timer/backend/app.py --mock \
  --host 127.0.0.1 --port 8443 \
  --ssl-certfile /path/to/localhost.pem \
  --ssl-keyfile /path/to/localhost-key.pem
```

3) In the frontend, set the bridge URL to `wss://localhost:8443/ws`.

Notes:
- Browsers must trust the certificate (mkcert installs a local CA).
- Corporate-managed PCs may restrict custom CAs.
