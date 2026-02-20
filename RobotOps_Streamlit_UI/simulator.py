"""
Simulator: generates realistic AI messages for Robot1, Controller, and Helper.
Used for demo and testing without live MQTT/ROS.
"""
import random
import uuid
from datetime import datetime
from typing import Optional

from models import Message, SourceType


# Pool of messages for each source and severity
ROBOT1_INFO = [
    "Path clear; obstacle at 1.8m",
    "Path clear; obstacle at 2.1m",
    "Navigating normally; nearest obstacle 2.5m",
    "LiDAR scan nominal; no close obstacles",
]
ROBOT1_WARN = [
    "LiDAR scan degraded",
    "Reduced scan rate; checking connection",
    "Obstacle at 0.9m; proceeding with caution",
    "Vision latency elevated",
]
ROBOT1_CRITICAL = [
    "Obstacle at 0.3m; emergency halt recommended",
    "Immediate stop: obstacle 0.2m ahead",
    "CRITICAL: obstacle in path at 0.25m",
]
CONTROLLER_MSGS = [
    "Task assigned to helper robot",
    "Robot1 continue; helper standby",
    "Re-routing Robot1; obstacle ahead",
    "All clear; proceed.",
]
HELPER_MSGS = [
    "Task acknowledged",
    "Moving to waypoint",
    "Standing by",
    "Task complete",
]


def _make_message(
    source: SourceType,
    severity: str,
    text: str,
    vision: Optional[dict] = None,
    lidar: Optional[dict] = None,
    action: Optional[str] = None,
) -> Message:
    return Message(
        id=str(uuid.uuid4()),
        ts=datetime.now(),
        source=source,
        severity=severity,
        text=text,
        vision=vision,
        lidar=lidar,
        action=action,
    )


def generate_robot1_info(simulate_lidar_stall: bool = False) -> Message:
    text = random.choice(ROBOT1_INFO)
    lidar = {"scan_hz": 0.0 if simulate_lidar_stall else round(random.uniform(6.5, 7.5), 1), "min_dist_m": 1.8}
    return _make_message("robot1", "info", text, lidar=lidar)


def generate_robot1_warn(simulate_lidar_stall: bool = False) -> Message:
    text = random.choice(ROBOT1_WARN)
    lidar = {"scan_hz": 0.0 if simulate_lidar_stall else round(random.uniform(3, 5), 1), "min_dist_m": 0.9}
    return _make_message("robot1", "warn", text, lidar=lidar)


def generate_robot1_critical() -> Message:
    text = random.choice(ROBOT1_CRITICAL)
    return _make_message(
        "robot1", "critical", text,
        vision={"summary": "Obstacle detected in frame"},
        lidar={"scan_hz": 7.0, "min_dist_m": 0.3},
        action="Emergency halt recommended",
    )


def generate_controller() -> Message:
    text = random.choice(CONTROLLER_MSGS)
    return _make_message("controller", "info", text, action=text)


def generate_helper() -> Message:
    text = random.choice(HELPER_MSGS)
    return _make_message("helper", "info", text, action=text)


def run_simulator_step(
    simulate_robot1_down: bool,
    simulate_lidar_stall: bool,
    simulate_critical_burst: bool,
) -> Optional[Message]:
    """
    Generate one message every 1–3 sec (call from app on timer).
    Returns one Message or None.
    """
    if simulate_robot1_down:
        # No Robot1 messages when "down"
        choice = random.choice(["controller", "controller", "helper"])
    else:
        choice = random.choice(["robot1", "robot1", "controller", "helper"])

    if choice == "robot1":
        if simulate_critical_burst:
            return generate_robot1_critical()
        r = random.random()
        if r < 0.1:
            return generate_robot1_critical()
        if r < 0.35:
            return generate_robot1_warn(simulate_lidar_stall)
        return generate_robot1_info(simulate_lidar_stall)
    if choice == "controller":
        return generate_controller()
    if choice == "helper":
        return generate_helper()
    return None
