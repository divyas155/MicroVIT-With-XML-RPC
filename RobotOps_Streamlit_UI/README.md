# RobotOps Streamlit UI

Monitoring and troubleshooting dashboard for the multi-robot system. Presents **Robot1** as one unified entity (Orin + Nano).

## Run

```bash
cd RobotOps_Streamlit_UI
pip install -r requirements.txt
streamlit run app.py
```

Open the URL shown (default `http://localhost:8501`).

## Features

- **Dashboard** – Tiles for Robot1, Controller, Helper Robot (status, last message, last seen, severity). Buttons: View Live Feed, Show Critical Only, Simulate Failure. Plotly chart of messages by source.
- **Live Feed** – Streaming table (auto-refresh every 2 s): Time, Source, Severity, Message. Filters and search. Row index selects message detail (vision, LiDAR, action, raw JSON).
- **Robot1 Detail** – Unified view: AI Perception (Orin), Navigation & LiDAR (Nano), Suggested Actions, Recent Alerts.
- **Troubleshoot** – Alerts with meaning, probable causes, step-by-step checks, recommended commands (`rostopic hz /scan`, `rosnode list`, `ping <ip>`, etc.).
- **Simulator** – Toggles: Simulate Robot1 Down, LiDAR Stall, Critical Burst. Generates realistic messages every 1–3 s.

## Data model

Messages use Pydantic `Message`: `id`, `ts`, `source` (robot1 | controller | helper), `severity`, `text`, `vision`, `lidar`, `action`. Session state holds messages, alerts, robot health, and simulator flags.

## Live real-time messages (real system)

To see **live** messages from your Controller, Robot1 (Orin), and Helper:

1. Start the real system (Controller Pi → Nano bringup → Orin Ollama → Orin Robot1 AI) as in **COMPLETE_SYSTEM_START.md**.
2. Run the Streamlit app, then in the sidebar check **Use Live MQTT** and set the broker host to your Controller Pi IP (e.g. **10.13.68.48**), port **1883**.
3. Open **Dashboard** or **Live Feed** to see real-time messages.

Full step-by-step: **LIVE_MQTT_STEPS.md**.
