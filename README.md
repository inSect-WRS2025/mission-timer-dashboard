# Mission Timer & Scoreboard (Operation Dashboard)

This repository hosts the offline-first dashboard to support decision-making during the WRS2025 Simulation Disaster Challenge.

- Single-page web app (no external deps) under `docs/` for GitHub Pages.
- Optional backend (FastAPI) for ROS 2 read-only events under `apps/mission-timer/backend/`.
- Design docs under `Design/`.

## Quick Start
- Open `docs/index.html` in a Chromium-based browser to run fully offline.
- Optional backend (mock events):
  ```bash
  python apps/mission-timer/backend/app.py --mock
  ```
  Then set Bridge URL in the UI to `ws://localhost:8000/ws`.

## GitHub Pages
- Enable Pages from the repository settings and select "Deploy from a branch" → `main` → `/docs`.
- Note: When the page is served over HTTPS, browsers may block `ws://` as mixed content. Use a local WSS endpoint. Planned: backend TLS flags `--ssl-certfile`/`--ssl-keyfile`, then connect to `wss://localhost:8443/ws`.

## Structure
- `docs/` – static SPA (Timer & Scoreboard UI)
- `apps/mission-timer/frontend/` – source of the SPA
- `apps/mission-timer/backend/` – FastAPI + WebSocket (optional ROS 2 bridge)
- `Design/` – specs (MVP and dashboard vision)

## References
- MVP spec: `Design/MissionTimerScoreboard.md`
- Dashboard vision/roadmap: `Design/MissionTimerDashboard_Vision.md`

