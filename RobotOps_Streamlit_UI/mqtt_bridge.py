"""
MQTT bridge: subscribe to live topics and convert payloads to Message-like dicts.
Used by the Streamlit app to show real-time messages from Controller, Robot1 (Orin), Helper.
"""
from __future__ import annotations

import json
import threading
import uuid
from collections import deque
from datetime import datetime
from typing import Any

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

# Max messages to keep in memory (drop oldest)
MAX_QUEUE = 500


def _robot_id_to_source(robot_id: str) -> str:
    if not robot_id:
        return "robot1"
    rid = (robot_id or "").lower()
    if rid in ("jetson2", "helper_robot", "helper"):
        return "helper"
    return "robot1"  # jetson1, robot1_orin, etc.


def _parse_obstacle(payload: str, topic: str) -> dict[str, Any] | None:
    try:
        data = json.loads(payload)
    except Exception:
        return None
    # topic: robots/<robot_id>/obstacle
    parts = topic.split("/")
    robot_id = parts[1] if len(parts) >= 2 else "jetson1"
    source = _robot_id_to_source(robot_id)
    natural = data.get("natural_message") or data.get("message") or str(data.get("event", "obstacle_detected"))
    severity = (data.get("severity") or "info").lower()
    if severity not in ("info", "warn", "critical"):
        severity = "info"
    ts_str = data.get("time") or data.get("timestamp")
    try:
        ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00")) if ts_str else datetime.now()
    except Exception:
        ts = datetime.now()
    lidar = data.get("lidar_data") or data.get("lidar")
    return {
        "id": str(uuid.uuid4()),
        "ts": ts,
        "source": source,
        "severity": severity,
        "text": natural[:2000] if isinstance(natural, str) else str(natural)[:2000],
        "vision": data.get("vision") or ({"summary": data.get("message_type", "")} if data.get("message_type") else None),
        "lidar": lidar,
        "action": data.get("proposed_action"),
    }


def _parse_ai_decision(payload: str) -> dict[str, Any] | None:
    try:
        data = json.loads(payload)
    except Exception:
        return None
    msg = data.get("message") or data.get("human_message") or f"Action: {data.get('action_taken', '')} for {data.get('target_robot', '')}"
    ts_str = data.get("timestamp")
    try:
        ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00")) if ts_str else datetime.now()
    except Exception:
        ts = datetime.now()
    return {
        "id": str(uuid.uuid4()),
        "ts": ts,
        "source": "controller",
        "severity": "info",
        "text": msg[:2000] if isinstance(msg, str) else str(msg)[:2000],
        "vision": None,
        "lidar": None,
        "action": data.get("action_taken"),
    }


class MQTTBridge:
    def __init__(self, host: str, port: int = 1883, username: str = "", password: str = ""):
        self.host = host
        self.port = port
        self.username = username
        self.password = password
        self._client: Any = None
        self._queue: deque = deque(maxlen=MAX_QUEUE)
        self._lock = threading.Lock()
        self._connected = False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            client.subscribe("robots/+/obstacle", qos=1)
            client.subscribe("controller/ai_decision", qos=1)
        else:
            self._connected = False

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace")
        if topic.endswith("/obstacle"):
            parsed = _parse_obstacle(payload, topic)
        elif topic == "controller/ai_decision":
            parsed = _parse_ai_decision(payload)
        else:
            parsed = None
        if parsed:
            with self._lock:
                self._queue.append(parsed)

    def start(self) -> bool:
        if mqtt is None:
            return False
        if self._client is not None:
            return self._connected
        self._client = mqtt.Client()
        if self.username or self.password:
            self._client.username_pw_set(self.username, self.password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        try:
            self._client.connect(self.host, self.port, 60)
            self._client.loop_start()
            return True
        except Exception:
            return False

    def stop(self):
        if self._client:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass
            self._client = None
        self._connected = False

    def get_pending_messages(self) -> list[dict]:
        with self._lock:
            out = list(self._queue)
            self._queue.clear()
        return out

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._connected
