"""
RobotOps Gateway: MQTT → Loki (logs) + Prometheus /metrics.
Unified entity: Robot1 (Nano+Orin). Topics: robots/robot1/status, robots/controller/status, robots/helper/status, robots/alerts.
"""
import json
import logging
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any

import requests
from dotenv import load_dotenv
from fastapi import FastAPI
from prometheus_client import REGISTRY, Counter, Gauge, generate_latest
from prometheus_client.openmetrics.exposition import CONTENT_TYPE_LATEST
from paho.mqtt import client as mqtt

load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

MQTT_HOST = os.getenv("MQTT_BROKER_HOST", "mosquitto")
MQTT_PORT = int(os.getenv("MQTT_BROKER_PORT", "1883"))
LOKI_URL = os.getenv("LOKI_URL", "http://loki:3100").rstrip("/")

# Prometheus metrics
robot_last_seen_seconds = Gauge("robot_last_seen_seconds", "Last message time (unix)", ["source"])
robot_status = Gauge("robot_status", "0=DOWN 1=OK 2=WARN", ["source"])
robot_messages_total = Counter("robot_messages_total", "Total messages", ["source"])
robot_critical_messages_total = Counter("robot_critical_messages_total", "Critical messages", ["source"])
robot_lidar_scan_hz = Gauge("robot_lidar_scan_hz", "LiDAR scan Hz (Robot1)", ["source"])
robot_obstacle_distance_m = Gauge("robot_obstacle_distance_m", "Obstacle distance m (Robot1)", ["source"])
robot_network_latency_ms = Gauge("robot_network_latency_ms", "Network latency ms", ["source"])
robot_packet_loss_pct = Gauge("robot_packet_loss_pct", "Packet loss %", ["source"])

SOURCES = ("robot1", "controller", "helper")
STATUS_OK, STATUS_WARN, STATUS_DOWN = 1, 2, 0
DOWN_THRESHOLD_SEC = 30

# In-memory state for metrics (updated by MQTT callback)
_state = {
    "last_seen": {s: 0.0 for s in SOURCES},
    "status": {s: STATUS_DOWN for s in SOURCES},
    "lidar_hz": {"robot1": 0.0},
    "obstacle_m": {"robot1": 0.0},
    "latency_ms": {s: 0.0 for s in SOURCES},
    "packet_loss": {s: 0.0 for s in SOURCES},
}
_lock = threading.Lock()

app = FastAPI(title="RobotOps Gateway")
mqtt_client: mqtt.Client | None = None


def topic_to_source(topic: str) -> str:
    if "robot1" in topic:
        return "robot1"
    if "controller" in topic:
        return "controller"
    if "helper" in topic:
        return "helper"
    return "robot1"


def push_to_loki(source: str, severity: str, log_line: str, ts_ns: int | None = None):
    if ts_ns is None:
        ts_ns = int(time.time() * 1e9)
    payload = {
        "streams": [
            {
                "stream": {
                    "app": "robotops",
                    "source": source,
                    "severity": severity.upper(),
                },
                "values": [[str(ts_ns), log_line]],
            }
        ]
    }
    try:
        r = requests.post(
            f"{LOKI_URL}/loki/api/v1/push",
            json=payload,
            timeout=2,
            headers={"Content-Type": "application/json"},
        )
        if r.status_code >= 400:
            logger.warning("Loki push %s: %s", r.status_code, r.text[:200])
    except Exception as e:
        logger.warning("Loki push error: %s", e)


def update_metrics():
    now = time.time()
    with _lock:
        for s in SOURCES:
            last = _state["last_seen"][s]
            if last > 0 and (now - last) > DOWN_THRESHOLD_SEC:
                _state["status"][s] = STATUS_DOWN
            robot_last_seen_seconds.labels(source=s).set(last)
            robot_status.labels(source=s).set(_state["status"][s])
            robot_network_latency_ms.labels(source=s).set(_state["latency_ms"][s])
            robot_packet_loss_pct.labels(source=s).set(_state["packet_loss"][s])
        robot_lidar_scan_hz.labels(source="robot1").set(_state["lidar_hz"].get("robot1", 0))
        robot_obstacle_distance_m.labels(source="robot1").set(_state["obstacle_m"].get("robot1", 0))


def on_mqtt_message(client, userdata, msg):
    topic = msg.topic
    source = topic_to_source(topic)
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        data = {"text": msg.payload.decode("utf-8", errors="replace")}

    ts = time.time()
    severity = (data.get("severity") or "INFO").upper()
    if severity not in ("INFO", "WARN", "ERROR", "CRITICAL"):
        severity = "INFO"
    title = data.get("title") or ""
    text = data.get("text") or ""
    log_line = json.dumps({"ts": data.get("ts"), "title": title, "text": text[:500], "source": source, "severity": severity})
    if data.get("lidar"):
        log_line += " " + json.dumps(data["lidar"])
    ts_iso = data.get("ts") or datetime.now(tz=timezone.utc).isoformat()
    try:
        ts_ns = int(datetime.fromisoformat(ts_iso.replace("Z", "+00:00")).timestamp() * 1e9)
    except Exception:
        ts_ns = int(ts * 1e9)

    push_to_loki(source, severity, log_line, ts_ns)

    with _lock:
        _state["last_seen"][source] = ts
        _state["status"][source] = STATUS_OK
        if severity in ("WARN", "ERROR"):
            _state["status"][source] = STATUS_WARN
        robot_messages_total.labels(source=source).inc()
        if severity == "CRITICAL":
            robot_critical_messages_total.labels(source=source).inc()

        lidar = data.get("lidar")
        if lidar and isinstance(lidar, dict):
            if source == "robot1":
                _state["lidar_hz"]["robot1"] = float(lidar.get("scanHz", 0) or 0)
                _state["obstacle_m"]["robot1"] = float(lidar.get("distanceM", 0) or 0)
            robot_lidar_scan_hz.labels(source="robot1").set(_state["lidar_hz"].get("robot1", 0))
            robot_obstacle_distance_m.labels(source="robot1").set(_state["obstacle_m"].get("robot1", 0))

        net = data.get("network")
        if net and isinstance(net, dict):
            _state["latency_ms"][source] = float(net.get("latencyMs", 0) or 0)
            _state["packet_loss"][source] = float(net.get("packetLossPct", 0) or 0)

    update_metrics()
    logger.info("Msg %s %s: %s", source, severity, text[:80])


def mqtt_loop():
    global mqtt_client
    client = mqtt.Client()
    client.on_connect = lambda c, u, flags, rc: (
        c.subscribe("robots/robot1/status", 1),
        c.subscribe("robots/controller/status", 1),
        c.subscribe("robots/helper/status", 1),
        c.subscribe("robots/alerts", 1),
        logger.info("MQTT connected, subscribed"),
    )
    client.on_message = on_mqtt_message
    while True:
        try:
            client.connect(MQTT_HOST, MQTT_PORT, 60)
            client.loop_forever()
        except Exception as e:
            logger.warning("MQTT reconnect: %s", e)
        time.sleep(5)


@app.on_event("startup")
def startup():
    for s in SOURCES:
        robot_last_seen_seconds.labels(source=s).set(0)
        robot_status.labels(source=s).set(STATUS_DOWN)
        robot_network_latency_ms.labels(source=s).set(0)
        robot_packet_loss_pct.labels(source=s).set(0)
    robot_lidar_scan_hz.labels(source="robot1").set(0)
    robot_obstacle_distance_m.labels(source="robot1").set(0)
    thread = threading.Thread(target=mqtt_loop, daemon=True)
    thread.start()


@app.get("/health")
def health():
    return {"status": "ok", "service": "robotops-gateway"}


@app.get("/metrics")
def metrics():
    update_metrics()
    return generate_latest(REGISTRY), {"Content-Type": CONTENT_TYPE_LATEST}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
