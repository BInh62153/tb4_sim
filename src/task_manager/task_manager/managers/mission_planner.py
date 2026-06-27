#!/usr/bin/env python3
"""
managers/mission_planner.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MissionPlanner: load YAML config, quản lý patrol sequence.

Inject:
    logger   – ROS logger hoặc bất kỳ object có .info/.error
    clock    – rclpy Clock (dùng để sinh mission_id theo timestamp)
    params   – dict tham số (waypoints_file, loop_patrol, ...)
"""

from __future__ import annotations

import uuid
import yaml
from typing import Any, Dict, List, Optional


class MissionPlanner:
    """Load YAML waypoint config và quản lý trình tự patrol."""

    def __init__(self, logger, clock=None, params: Optional[Dict] = None):
        self._log            = logger
        self._clock          = clock
        self._params         = params or {}
        self.waypoints:       Dict[str, Any]  = {}
        self.patrol_sequence: List[str]       = []
        self.loop_patrol:     bool            = True
        self.current_idx:     int             = 0
        self.mission_id:      str             = ""
        self.dock_waypoint:   str             = "tram_sac"

    # ── Config ─────────────────────────────────────────────────────────────

    def load_config(self, filepath: str) -> bool:
        """Load YAML. Trả về False nếu lỗi."""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            if not data:
                raise ValueError("Empty YAML file")
            self.patrol_sequence = data.get('patrol_sequence', [])
            self.loop_patrol     = data.get('loop_patrol', True)
            self.waypoints       = data.get('waypoints', {})
            self.dock_waypoint   = (
                data.get('emergency_rules', {}).get('low_battery_action', 'tram_sac')
            )
            self.current_idx     = 0
            self.mission_id      = str(uuid.uuid4())[:8]
            self._log.info(
                f"[Mission] Loaded {len(self.waypoints)} waypoints, "
                f"{len(self.patrol_sequence)} in sequence. "
                f"mission_id={self.mission_id}"
            )
            return True
        except Exception as exc:
            self._log.error(f"[Mission] Failed to load config '{filepath}': {exc}")
            return False

    # ── Sequence control ───────────────────────────────────────────────────

    def get_current_waypoint_name(self) -> Optional[str]:
        if not self.patrol_sequence:
            return None
        if self.current_idx >= len(self.patrol_sequence):
            return None
        return self.patrol_sequence[self.current_idx]

    def get_current_waypoint_data(self) -> Optional[Dict]:
        name = self.get_current_waypoint_name()
        if name is None:
            return None
        return self.waypoints.get(name)

    def advance_mission(self):
        self.current_idx += 1
        if self.current_idx >= len(self.patrol_sequence):
            if self.loop_patrol:
                self.current_idx = 0
                self._log.info("[Mission] Patrol loop completed. Resetting.")
            else:
                self._log.info("[Mission] Patrol sequence finished.")

    def reset(self):
        self.current_idx = 0
        self.mission_id  = str(uuid.uuid4())[:8]

    def get_dock_waypoint_data(self) -> Optional[Dict]:
        return self.waypoints.get(self.dock_waypoint)

    # ── Status ─────────────────────────────────────────────────────────────

    def is_mission_complete(self) -> bool:
        return (
            not self.loop_patrol
            and self.current_idx >= len(self.patrol_sequence)
        )

    def status_summary(self) -> str:
        return (
            f"mission={self.mission_id} "
            f"wp={self.get_current_waypoint_name() or 'done'} "
            f"idx={self.current_idx}/{len(self.patrol_sequence)}"
        )
