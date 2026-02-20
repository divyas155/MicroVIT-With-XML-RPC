"""
Pydantic models for RobotOps Streamlit UI.
Message and related structures for validation and display.
"""
from datetime import datetime
from typing import Optional, Literal
from pydantic import BaseModel

SourceType = Literal["robot1", "controller", "helper"]
SeverityType = Literal["info", "warn", "critical"]


class Message(BaseModel):
    id: str
    ts: datetime
    source: SourceType
    severity: str = "info"
    text: str
    vision: Optional[dict] = None
    lidar: Optional[dict] = None
    action: Optional[str] = None

    class Config:
        json_encoders = {datetime: lambda v: v.isoformat()}


class RobotHealth(BaseModel):
    source: SourceType
    status: Literal["OK", "WARN", "DOWN"] = "OK"
    last_message: str = ""
    last_seen: Optional[datetime] = None
    severity_badge: str = "info"


class Alert(BaseModel):
    id: str
    rule: str  # SOURCE_DOWN | LIDAR_STALL | CRITICAL_MSG
    source: SourceType
    meaning: str
    probable_causes: list[str] = []
    checks: list[str] = []
    commands: list[str] = []
    raised_at: Optional[datetime] = None

    def model_post_init(self, __context):
        if self.raised_at is None:
            object.__setattr__(self, "raised_at", datetime.now())
