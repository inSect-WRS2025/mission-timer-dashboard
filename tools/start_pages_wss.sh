#!/usr/bin/env bash
set -euo pipefail

# Start the local backend with TLS (WSS) so a GitHub Pages (HTTPS) frontend
# can connect to ws endpoint at wss://127.0.0.1:8443/ws
#
# Usage:
#   tools/start_pages_wss.sh              # mock mode, auto-detect certs
#   tools/start_pages_wss.sh --dry-run    # no server start, just planned settings
#   USE_MOCK=0 tools/start_pages_wss.sh   # try to use config.yaml if present
#   PORT=9443 HOST=127.0.0.1 tools/start_pages_wss.sh
#   CERT=./localhost.pem KEY=./localhost-key.pem tools/start_pages_wss.sh
#
# Requirements:
#   - Python 3.9+
#   - pip install fastapi uvicorn pyyaml  (not required for --dry-run)
#   - mkcert (optional) to generate local TLS certs for localhost

HERE="$(cd -- "$(dirname -- "$0")" && pwd)"
ROOT="$(cd -- "${HERE}/.." && pwd)"   # mission-timer-dashboard

PY=python3
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8443}"
USE_MOCK="${USE_MOCK:-1}"
CERT="${CERT:-}"
KEY="${KEY:-}"

WRAPPER="${ROOT}/tools/run_bridge_wss.py"
BACKEND_CFG_DEFAULT="${ROOT}/apps/mission-timer/backend/config.yaml"

if ! command -v "$PY" >/dev/null 2>&1; then
  echo "[error] python3 not found in PATH." >&2
  exit 1
fi

# Try to find certs if not provided
if [[ -z "$CERT" || -z "$KEY" ]]; then
  if [[ -f "${ROOT}/localhost+2.pem" && -f "${ROOT}/localhost+2-key.pem" ]]; then
    CERT="${ROOT}/localhost+2.pem"
    KEY="${ROOT}/localhost+2-key.pem"
  elif [[ -f "${ROOT}/localhost.pem" && -f "${ROOT}/localhost-key.pem" ]]; then
    CERT="${ROOT}/localhost.pem"
    KEY="${ROOT}/localhost-key.pem"
  fi
fi

ARGS=("$WRAPPER" "--host" "$HOST" "--port" "$PORT")

# Use config if requested and present; otherwise mock
if [[ "${USE_MOCK}" == "0" && -f "$BACKEND_CFG_DEFAULT" ]]; then
  ARGS+=("--config" "$BACKEND_CFG_DEFAULT")
else
  ARGS+=("--mock")
fi

# TLS if certs available
if [[ -n "$CERT" && -n "$KEY" ]]; then
  ARGS+=("--certfile" "$CERT" "--keyfile" "$KEY")
else
  echo "[warn] TLS certs not found. Will start plain WS (Pages cannot connect)." >&2
fi

# Pass through any extra CLI args (e.g., --dry-run)
ARGS+=("$@")

echo "[info] Launching backend via: $PY ${ARGS[*]}"
"$PY" "${ARGS[@]}"

STATUS=$?
if [[ $STATUS -ne 0 ]]; then
  echo "[error] Backend exited with status $STATUS" >&2
  echo "[hint] If missing, install deps: pip install fastapi uvicorn pyyaml" >&2
fi
exit $STATUS

