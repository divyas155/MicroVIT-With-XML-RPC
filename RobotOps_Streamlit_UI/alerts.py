"""
Alert rules and fusion logic.
- SOURCE_DOWN: no message from source for > 30 sec
- LIDAR_STALL: scanHz 0 or null in recent Robot1 message
- CRITICAL_MSG: severity critical
"""
from datetime import datetime, timedelta
from typing import List

from models import Message, Alert, SourceType

SOURCE_DOWN_SEC = 30


def _robot1_last_seen(messages: List[Message]) -> datetime | None:
    for m in reversed(messages):
        if m.source == "robot1":
            return m.ts
    return None


def _controller_last_seen(messages: List[Message]) -> datetime | None:
    for m in reversed(messages):
        if m.source == "controller":
            return m.ts
    return None


def _helper_last_seen(messages: List[Message]) -> datetime | None:
    for m in reversed(messages):
        if m.source == "helper":
            return m.ts
    return None


def _robot1_latest_lidar_hz(messages: List[Message]) -> float | None:
    for m in reversed(messages):
        if m.source == "robot1" and m.lidar is not None:
            hz = m.lidar.get("scan_hz")
            if hz is not None:
                return float(hz)
    return None


def run_alert_rules(messages: List[Message], now: datetime) -> List[Alert]:
    alerts: List[Alert] = []
    seen_rules: set[str] = set()

    # SOURCE_DOWN
    for source in ["robot1", "controller", "helper"]:
        if source == "robot1":
            last = _robot1_last_seen(messages)
        elif source == "controller":
            last = _controller_last_seen(messages)
        else:
            last = _helper_last_seen(messages)
        if last is None:
            continue
        if (now - last).total_seconds() > SOURCE_DOWN_SEC:
            key = f"SOURCE_DOWN_{source}"
            if key not in seen_rules:
                seen_rules.add(key)
                a = Alert(
                    id=key,
                    rule="SOURCE_DOWN",
                    source=source,
                    meaning=f"{source.upper()} has not sent a message in over {SOURCE_DOWN_SEC} seconds.",
                    probable_causes=[
                        "Service or node stopped on that unit.",
                        "Network or MQTT disconnect.",
                        "Unit powered off or unreachable.",
                    ],
                    checks=[
                        f"Check if {source} process is running.",
                        "Check MQTT broker and subscriptions.",
                        "Check network connectivity.",
                    ],
                    commands=[
                        "rosnode list",
                        "rostopic list",
                        f"ping <{source}_ip>",
                    ],
                )
                alerts.append(a)

    # LIDAR_STALL: scan_hz 0 or no lidar data in recent Robot1 messages
    hz = _robot1_latest_lidar_hz(messages)
    robot1_with_lidar = any(m.source == "robot1" and m.lidar for m in messages)
    if robot1_with_lidar and (hz is None or hz == 0):
        if "LIDAR_STALL" not in seen_rules:
            seen_rules.add("LIDAR_STALL")
            alerts.append(Alert(
                id="LIDAR_STALL",
                rule="LIDAR_STALL",
                source="robot1",
                meaning="Robot1 LiDAR scan rate is 0 or missing (LiDAR not publishing).",
                probable_causes=[
                    "rplidarNode not running or crashed.",
                    "Wrong baud rate (e.g. 256000 vs 115200).",
                    "USB port or cable issue on Nano.",
                ],
                checks=[
                    "Check rplidarNode is running on Nano.",
                    "Check /scan topic has data.",
                    "Verify LiDAR serial port and baud.",
                ],
                commands=[
                    "rostopic hz /scan",
                    "rosnode list",
                    "rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=115200",
                ],
            ))

    # CRITICAL_MSG (per-message; we add one representative alert)
    has_critical = any(m.severity == "critical" for m in messages)
    if has_critical and "CRITICAL_MSG" not in seen_rules:
        seen_rules.add("CRITICAL_MSG")
        alerts.append(Alert(
            id="CRITICAL_MSG",
            rule="CRITICAL_MSG",
            source="robot1",
            meaning="A critical-severity message was generated (e.g. obstacle very close).",
            probable_causes=[
                "Obstacle detected within emergency distance.",
                "Sensor or AI flagged an urgent condition.",
            ],
            checks=[
                "Confirm obstacle location and clear if safe.",
                "Check LiDAR and vision data for the robot.",
            ],
            commands=[
                "rostopic echo /scan",
                "Review Robot1 detail page for suggested actions.",
            ],
        ))

    return alerts


def get_robot_health(messages: List[Message], now: datetime) -> dict[SourceType, dict]:
    """Fusion: compute status, last message, last seen, severity for Robot1, Controller, Helper."""
    health = {}
    for source in ["robot1", "controller", "helper"]:
        src = source
        last_msg = None
        for m in reversed(messages):
            if m.source == src:
                last_msg = m
                break
        if last_msg is None:
            health[src] = {
                "status": "DOWN",
                "last_message": "",
                "last_seen": None,
                "severity_badge": "critical",
            }
            continue
        age_sec = (now - last_msg.ts).total_seconds()
        if age_sec > SOURCE_DOWN_SEC:
            status = "DOWN"
            severity_badge = "critical"
        elif last_msg.severity == "critical" or age_sec > 15:
            status = "WARN"
            severity_badge = "warn" if last_msg.severity != "critical" else "critical"
        else:
            status = "OK"
            severity_badge = last_msg.severity
        health[src] = {
            "status": status,
            "last_message": last_msg.text[:80] + ("..." if len(last_msg.text) > 80 else ""),
            "last_seen": last_msg.ts,
            "severity_badge": severity_badge,
        }
    return health
