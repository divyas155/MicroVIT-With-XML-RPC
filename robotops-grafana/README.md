# RobotOps Observability Stack (Grafana)

Ops-room style monitoring for a multi-robot system using **Grafana**, **Loki**, **Prometheus**, and a lightweight **gateway**. **Robot1** is a single unified entity (Jetson Nano + Jetson Orin); no separate Nano/Orin tiles.

Runs **offline on a closed LAN**; no cloud dependencies.

---

## Quick start

```bash
cd robotops-grafana
docker compose up -d
```

- **Grafana:** http://localhost:3000 — login `admin` / `admin`
- **Prometheus:** http://localhost:9090
- **Gateway health:** http://localhost:8080/health
- **MQTT (optional in stack):** local broker on port 1883

Datasources (Prometheus, Loki) and three dashboards are provisioned automatically. Open **Dashboards → RobotOps** and choose:

- **RobotOps — Command Overview** — status tiles, messages/min, CRITICAL/ERROR logs, latest messages
- **RobotOps — Robot1 Deep Dive (Unified)** — Robot1 last seen, msg rate, LiDAR Hz, obstacle distance, Robot1 logs
- **RobotOps — Alerts & Runbooks** — active alerts, log evidence, copy-paste runbook commands

---

## Architecture

```
MQTT (robots/robot1/status, robots/controller/status, robots/helper/status, robots/alerts)
    → Gateway (subscribes, pushes logs to Loki, exposes /metrics)
         → Loki (log store)
         → Prometheus (scrapes /metrics, evaluates alert rules)
              → Grafana (dashboards + alerts)
```

- **Gateway:** Python FastAPI; subscribes to MQTT; pushes each message to Loki with labels `app=robotops`, `source`, `severity`; exposes Prometheus metrics at `/metrics`.
- **Loki:** Stores log lines; Grafana queries by label and time.
- **Prometheus:** Scrapes gateway every 10s; runs alert rules (SOURCE_DOWN, LIDAR_STALL, CRITICAL_MSG, HIGH_LATENCY).
- **Grafana:** Pre-provisioned datasources and three dashboards; dark theme, ops-room style.

---

## Publishing test data (sample publisher)

With the stack running and Mosquitto in the same compose (default):

```bash
pip install paho-mqtt
python scripts/publish_sample_messages.py --host localhost --port 1883
```

Messages are published every 1–3 seconds to `robots/robot1/status`, `robots/controller/status`, `robots/helper/status` with INFO/WARN/CRITICAL samples. Stop with Ctrl+C.

To use an **external MQTT broker** (e.g. your Controller Pi):

1. Set env before `docker compose up`: `export MQTT_BROKER_HOST=10.13.68.48` (and optionally `MQTT_BROKER_PORT=1883`).
2. Do **not** start the `mosquitto` service, or use `docker compose up -d gateway loki prometheus grafana` (omit mosquitto).
3. Run the sample publisher (or your real system) pointing at that broker:  
   `python scripts/publish_sample_messages.py --host 10.13.68.48 --port 1883`

---

## Message schema (MQTT JSON)

Publish JSON to the topics above in this shape (gateway parses and forwards to Loki / metrics):

```json
{
  "ts": "2025-02-15T12:00:00Z",
  "source": "robot1",
  "severity": "INFO",
  "title": "Path clear",
  "text": "Obstacle at 1.8m",
  "lidar": { "distanceM": 1.8, "scanHz": 7.2, "confidence": 0.95 },
  "network": { "latencyMs": 12, "packetLossPct": 0 },
  "actionSuggestion": null
}
```

- **robot1** (unified): use `lidar.distanceM`, `lidar.scanHz` for Robot1 metrics; gateway exports `robot_lidar_scan_hz`, `robot_obstacle_distance_m` for `source=robot1` only.

---

## Prometheus metrics (gateway)

| Metric | Type | Labels | Description |
|--------|------|--------|-------------|
| `robot_last_seen_seconds` | Gauge | source | Last message time (unix) |
| `robot_status` | Gauge | source | 0=DOWN, 1=OK, 2=WARN |
| `robot_messages_total` | Counter | source | Total messages |
| `robot_critical_messages_total` | Counter | source | Critical messages |
| `robot_lidar_scan_hz` | Gauge | source=robot1 | LiDAR scan Hz |
| `robot_obstacle_distance_m` | Gauge | source=robot1 | Obstacle distance (m) |
| `robot_network_latency_ms` | Gauge | source | Latency (ms) |
| `robot_packet_loss_pct` | Gauge | source | Packet loss (%) |

---

## Alert rules (Prometheus)

- **SOURCE_DOWN:** No message from robot1/controller/helper for 30+ seconds.
- **LIDAR_STALL:** Robot1 LiDAR scan Hz &lt; 0.1 (avg 10s).
- **CRITICAL_MSG:** Robot1 critical message count increased in 1m.
- **HIGH_LATENCY:** Network latency &gt; 300 ms (avg 30s) for 2m.

Rules are in `prometheus/rules/robotops_rules.yml`. Alerts appear in Grafana (Alerting) and on the **Alerts & Runbooks** dashboard.

---

## Adding new topics or metrics

1. **New MQTT topic:** In `gateway/main.py`, add a subscription in `on_connect` and map topic → source in `topic_to_source()` (or extend the callback). Keep **Robot1** as a single source (e.g. map both `robots/jetson1/obstacle` and `robots/robot1/status` to `robot1` if you want).
2. **New metric:** Add a `Gauge`/`Counter` in `gateway/main.py`, update it in `on_mqtt_message` from the payload, and expose it in `update_metrics()`.
3. **New alert:** Add a rule in `prometheus/rules/robotops_rules.yml`, then `curl -X POST http://localhost:9090/-/reload` if Prometheus has `--web.enable-lifecycle`, or restart the stack.
4. **New dashboard:** Create a JSON in `grafana/dashboards/` or duplicate and edit an existing one; it will be auto-loaded. Use datasource UIDs `prometheus` and `loki`.

---

## Files layout

```
robotops-grafana/
  docker-compose.yml       # grafana, loki, prometheus, gateway, mosquitto
  gateway/
    main.py                # MQTT → Loki + /metrics
    requirements.txt
    Dockerfile
  grafana/
    provisioning/
      datasources/datasources.yml
      dashboards/dashboards.yml
    dashboards/
      command_overview.json
      robot1_deep_dive.json
      alerts_runbooks.json
  prometheus/
    prometheus.yml
    rules/robotops_rules.yml
  scripts/
    publish_sample_messages.py
  mosquitto/
    mosquitto.conf
  README.md
```

---

## Default credentials

- **Grafana:** admin / admin (change after first login if not on a closed LAN).

This stack does **not** modify your existing project (Controller, Nano, Orin, Streamlit). It is a separate observability layer that consumes MQTT (or the sample publisher) and displays Robot1 as one unified entity.
