#!/usr/bin/env python3
"""
managers/battery_manager.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
BatteryManager: theo dõi pin, emit events khi pin thấp / đã sạc xong.

Thresholds (inject qua params dict):
    low_battery_pct      – mặc định 20 %
    resume_battery_pct   – mặc định 80 % (khi nào coi là đã sạc đủ)
    critical_battery_pct – mặc định 10 % (dừng khẩn cấp)
"""

from __future__ import annotations

from typing import Callable, Optional

from sensor_msgs.msg import BatteryState


class BatteryManager:
    """Subscriber /battery_state; gọi callbacks khi vượt ngưỡng."""

    DEFAULT_LOW       = 20.0
    DEFAULT_RESUME    = 80.0
    DEFAULT_CRITICAL  = 10.0

    def __init__(self, node, slog, params: Optional[dict] = None):
        self._node     = node
        self._slog     = slog
        p              = params or {}
        self.low_pct      = float(p.get('low_battery_pct',      self.DEFAULT_LOW))
        self.resume_pct   = float(p.get('resume_battery_pct',   self.DEFAULT_RESUME))
        self.critical_pct = float(p.get('critical_battery_pct', self.DEFAULT_CRITICAL))

        self.is_low      = False
        self.is_critical = False
        self.is_charging = False
        self.percentage  = 100.0

        self._sub            = None
        self._on_low_cb:      Optional[Callable] = None
        self._on_critical_cb: Optional[Callable] = None
        self._on_charged_cb:  Optional[Callable] = None

    # ── Public API ──────────────────────────────────────────────────────────

    def start(
        self,
        on_low:      Optional[Callable] = None,
        on_critical:  Optional[Callable] = None,
        on_charged:   Optional[Callable] = None,
    ):
        """Bắt đầu subscribe /battery_state."""
        self._on_low_cb      = on_low
        self._on_critical_cb = on_critical
        self._on_charged_cb  = on_charged
        self._sub = self._node.create_subscription(
            BatteryState, '/battery_state', self._cb, 10
        )

    def stop(self):
        if self._sub:
            self._node.destroy_subscription(self._sub)
            self._sub = None

    def reset_flags(self):
        """Gọi sau khi sạc xong để bắt đầu lại chu kỳ giám sát."""
        self.is_low      = False
        self.is_critical = False

    def status_str(self) -> str:
        return (
            f"{self.percentage:.1f}% "
            f"{'[CHARGING]' if self.is_charging else ''}"
            f"{'[LOW]' if self.is_low else ''}"
            f"{'[CRITICAL]' if self.is_critical else ''}"
        )

    # ── Private ─────────────────────────────────────────────────────────────

    def _cb(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        self.percentage  = pct
        self.is_charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )

        # CRITICAL (ưu tiên cao nhất)
        if pct < self.critical_pct and not self.is_critical and not self.is_charging:
            self.is_critical = True
            self.is_low      = True
            self._slog.error("battery_critical", pct=round(pct, 1))
            if self._on_critical_cb:
                self._on_critical_cb()

        # LOW
        elif pct < self.low_pct and not self.is_low and not self.is_charging:
            self.is_low = True
            self._slog.warn("battery_low", pct=round(pct, 1))
            if self._on_low_cb:
                self._on_low_cb()

        # Fully charged / resumed
        elif self.is_charging and pct >= self.resume_pct and self.is_low:
            self._slog.info("battery_charged", pct=round(pct, 1))
            self.reset_flags()
            if self._on_charged_cb:
                self._on_charged_cb()
