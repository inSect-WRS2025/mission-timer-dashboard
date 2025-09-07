#!/usr/bin/env python3
"""
Run the Mission Timer backend (FastAPI + WebSocket) with optional local TLS
so that a GitHub Pages (HTTPS) frontend can connect via WSS.

Usage examples:

  # 1) Mock mode with TLS (recommended for Pages)
  ./tools/run_bridge_wss.py --mock \
    --certfile ./localhost.pem --keyfile ./localhost-key.pem

  # 2) If no certs are provided/found, falls back to plain WS on port 8000
  ./tools/run_bridge_wss.py --mock

  # 3) Dry-run to only validate configuration
  ./tools/run_bridge_wss.py --mock --dry-run

Certificates: generate local trusted certs with mkcert
  mkcert -install
  mkcert localhost 127.0.0.1 ::1
  # e.g., localhost+2.pem and localhost+2-key.pem
"""

import argparse
import os
import sys
import types
from importlib.machinery import SourceFileLoader

# uvicorn is imported lazily in main() so --dry-run works without deps


def resolve_repo_root() -> str:
    # This script lives in <repo>/mission-timer-dashboard/tools/
    here = os.path.abspath(os.path.dirname(__file__))
    repo_root = os.path.abspath(os.path.join(here, os.pardir))  # mission-timer-dashboard
    return repo_root


def load_backend_module(repo_root: str):
    backend_path = os.path.join(repo_root, 'apps', 'mission-timer', 'backend', 'app.py')
    if not os.path.exists(backend_path):
        raise FileNotFoundError(f"backend app.py not found at {backend_path}")
    module_name = 'mission_timer_backend_app'
    loader = SourceFileLoader(module_name, backend_path)
    mod = types.ModuleType(module_name)
    loader.exec_module(mod)
    return mod


def guess_local_certs(cwd: str):
    # Try common mkcert outputs in CWD first, then repo root
    candidates = [
        ('localhost+2.pem', 'localhost+2-key.pem'),
        ('localhost.pem', 'localhost-key.pem'),
    ]
    for base in (cwd, resolve_repo_root()):
        for cert, key in candidates:
            c = os.path.join(base, cert)
            k = os.path.join(base, key)
            if os.path.exists(c) and os.path.exists(k):
                return c, k
    return None, None


def main():
    parser = argparse.ArgumentParser(description='Run local ROS2 bridge with optional TLS (WSS)')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8443, help='Port for TLS (default 8443) or WS fallback (8000)')
    parser.add_argument('--config', default=None, help='Path to YAML config for ROS2 topics')
    parser.add_argument('--mock', action='store_true', help='Run without ROS2 and generate mock events')
    parser.add_argument('--certfile', default=None, help='TLS certificate file (PEM)')
    parser.add_argument('--keyfile', default=None, help='TLS key file (PEM)')
    parser.add_argument('--dry-run', action='store_true', help='Validate and print planned settings, then exit')

    args = parser.parse_args()

    # Resolve TLS settings early so --dry-run does not require imports
    certfile = args.certfile
    keyfile = args.keyfile
    if not (certfile and keyfile):
        gc, gk = guess_local_certs(os.getcwd())
        if gc and gk:
            certfile, keyfile = gc, gk

    use_tls = bool(certfile and keyfile)

    # Adjust default port for non-TLS fallback
    host = args.host
    port = args.port if use_tls else (8000 if args.port == 8443 else args.port)

    scheme = 'wss' if use_tls else 'ws'
    url = f"{scheme}://{host}:{port}/ws"

    if args.dry_run:
        print('[dry-run] Backend app created OK.')
        print(f"[dry-run] Using TLS: {use_tls}")
        if use_tls:
            print(f"[dry-run] certfile: {certfile}")
            print(f"[dry-run] keyfile:  {keyfile}")
        else:
            print('[dry-run] No certs found/provided. Will run plain WS.')
        print(f"[dry-run] WebSocket endpoint: {url}")
        return 0

    # Import backend and create app only when actually running
    repo_root = resolve_repo_root()
    mod = load_backend_module(repo_root)

    # Prepare config
    cfg_dict = {}
    if args.config:
        import yaml  # lazy import
        with open(args.config, 'r') as f:
            cfg_dict = yaml.safe_load(f) or {}

    # Create app via backend factory
    app = mod.create_app(cfg_dict, use_mock=bool(args.mock))

    try:
        import uvicorn  # type: ignore
    except Exception as e:
        print(f"[error] uvicorn not available: {e}")
        print("[hint] Install with: pip install 'uvicorn[standard]' fastapi pyyaml")
        return 1

    # Ensure a WS implementation is present to avoid 'Unsupported upgrade request'
    ws_ok = True
    try:
        import websockets  # type: ignore
    except Exception:
        try:
            import wsproto  # type: ignore
        except Exception:
            ws_ok = False
    if not ws_ok:
        print("[error] No WebSocket transport installed for uvicorn.")
        print("[hint] Install one of: pip install 'uvicorn[standard]'  (recommended) or pip install websockets")
        return 1

    if use_tls:
        print(f"[info] Starting TLS server for GitHub Pages WSS: {url}")
        uvicorn.run(app, host=host, port=port, ssl_certfile=certfile, ssl_keyfile=keyfile)
    else:
        print('[warn] TLS certs not provided/found. Falling back to plain WS.')
        print(f"[warn] Pages(HTTPS) cannot connect to {url}; use local frontend or provide certs.")
        uvicorn.run(app, host=host, port=port)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
