#!/usr/bin/env python3
"""
Publish sample RobotOps messages to MQTT for testing the observability stack.
Unified entity: Robot1 only (no separate Nano/Orin). Run with stack up and mosquitto reachable.
Usage: pip install paho-mqtt; python publish_sample_messages.py [--host localhost] [--port 1883]
"""
import argparse
import json
import random
import time
from datetime import datetime, timezone

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Install: pip install paho-mqtt")
    raise

TOPICS = {
    "robot1": "robots/robot1/status",
    "controller": "robots/controller/status",
    "helper": "robots/helper/status",
}
ALERT_TOPIC = "robots/alerts"

SAMPLES = {
    "robot1": [
        {"severity": "INFO", "title": "Path clear", "text": "Path clear; obstacle at 1.8m", "lidar": {"distanceM": 1.8, "scanHz": 7.2, "confidence": 0.95}, "network": {"latencyMs": 12, "packetLossPct": 0}, "actionSuggestion": None},
        {"severity": "WARN", "title": "LiDAR degraded", "text": "LiDAR scan degraded", "lidar": {"distanceM": 0.9, "scanHz": 3.1, "confidence": 0.6}, "network": {"latencyMs": 45, "packetLossPct": 0.1}, "actionSuggestion": "Check /scan rate"},
        {"severity": "CRITICAL", "title": "Obstacle close", "text": "Obstacle at 0.3m; emergency halt recommended", "lidar": {"distanceM": 0.3, "scanHz": 7.0, "confidence": 0.99}, "network": {"latencyMs": 15, "packetLossPct": 0}, "actionSuggestion": "Halt and reassess"},
    ],
    "controller": [
        {"severity": "INFO", "title": "Task assigned", "text": "Task assigned to helper robot", "lidar": None, "network": {"latencyMs": 8, "packetLossPct": 0}, "actionSuggestion": None},
        {"severity": "INFO", "title": "Proceed", "text": "All clear; proceed.", "lidar": None, "network": {"latencyMs": 10, "packetLossPct": 0}, "actionSuggestion": None},
    ],
    "helper": [
        {"severity": "INFO", "title": "Ack", "text": "Task acknowledged", "lidar": None, "network": {"latencyMs": 5, "packetLossPct": 0}, "actionSuggestion": None},
    ],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost", help="MQTT broker host")
    ap.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    ap.add_argument("--interval", type=float, default=2.0, help="Seconds between messages (range 1-3)")
    args = ap.parse_args()

    client = mqtt.Client()
    client.connect(args.host, args.port, 60)
    client.loop_start()
    print(f"Publishing to {args.host}:{args.port} every 1-3 sec. Ctrl+C to stop.")
    try:
        while True:
            source = random.choice(["robot1", "controller", "helper"])
            payload = random.choice(SAMPLES[source]).copy()
            payload["ts"] = datetime.now(tz=timezone.utc).isoformat()
            payload["source"] = source
            topic = TOPICS[source]
            client.publish(topic, json.dumps(payload), qos=1)
            print(f"  {topic} [{payload['severity']}] {payload['text'][:50]}")
            time.sleep(random.uniform(1, max(1.5, args.interval)))
    except KeyboardInterrupt:
        pass
    client.loop_stop()
    client.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
