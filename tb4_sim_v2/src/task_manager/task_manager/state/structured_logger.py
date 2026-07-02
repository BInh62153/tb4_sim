#!/usr/bin/env python3
"""
Structured logging wrapper.

  slog = StructuredLogger(ros_logger, "mission_001")
  slog.info("nav_started", waypoint="diem_A", retry=0)
  # → [STRUCT] mission=mission_001 | event=nav_started | waypoint=diem_A | retry=0
"""

from __future__ import annotations
from typing import Optional


class StructuredLogger:
    """
    Ghi log có cấu trúc để sau này parse/phân tích dễ.
    Tất cả log đi qua đây đều có:
        mission_id, goal_id, waypoint, retry_count, recovery_level,
        nav_time, task_duration (optional).
    """

    def __init__(self, ros_logger, mission_id: str = ""):
        self._log        = ros_logger
        self.mission_id  = mission_id
        self.goal_id     = ""
        self.waypoint    = ""
        self.retry_count = 0
        self.recovery_level = 0

    def _fmt(self, event: str, **extra) -> str:
        parts = [
            f"mission={self.mission_id}",
            f"event={event}",
        ]
        if self.goal_id:
            parts.append(f"goal={self.goal_id}")
        if self.waypoint:
            parts.append(f"wp={self.waypoint}")
        if self.retry_count:
            parts.append(f"retry={self.retry_count}")
        if self.recovery_level:
            parts.append(f"recovery_lvl={self.recovery_level}")
        for k, v in extra.items():
            parts.append(f"{k}={v}")
        return "[STRUCT] " + " | ".join(parts)

    def info(self, event: str, **extra):
        self._log.info(self._fmt(event, **extra))

    def warn(self, event: str, **extra):
        self._log.warn(self._fmt(event, **extra))

    def error(self, event: str, **extra):
        self._log.error(self._fmt(event, **extra))

    def debug(self, event: str, **extra):
        self._log.debug(self._fmt(event, **extra))
