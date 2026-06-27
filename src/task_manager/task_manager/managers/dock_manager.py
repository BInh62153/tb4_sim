#!/usr/bin/env python3
"""
managers/dock_manager.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DockManager: navigate to dock, wait for charge, resume mission.

Flow:
    dock() → nav to dock_wp → on_arrived → wait/monitor battery
           → on_charged (from BatteryManager) → undock() → on_complete(True)

Inject:
    node         – ROS2 LifecycleNode
    nav_mgr      – NavigationManager
    battery_mgr  – BatteryManager
    slog         – StructuredLogger
"""

from __future__ import annotations

from typing import Callable, Dict, Optional


class DockManager:
    """Điều phối quá trình docking + charging + resume."""

    DOCK_TIMEOUT_S   = 120.0  # timeout nav to dock
    CHARGE_CHECK_S   = 10.0   # heartbeat khi đang sạc

    def __init__(self, node, nav_mgr, battery_mgr, slog):
        self._node        = node
        self._nav_mgr     = nav_mgr
        self._battery_mgr = battery_mgr
        self._slog        = slog

        self._on_complete:   Optional[Callable[[bool], None]] = None
        self._charge_timer  = None
        self._dock_wp_data: Optional[Dict] = None

    # ── Public API ──────────────────────────────────────────────────────────

    def start_docking(
        self,
        dock_wp_data: Optional[Dict],
        on_complete: Callable[[bool], None],
    ):
        """
        Bắt đầu docking sequence.
        on_complete(True) khi đã sạc xong và sẵn sàng tiếp tục mission.
        on_complete(False) khi docking thất bại sau retry.
        """
        self._on_complete = on_complete
        self._dock_wp_data = dock_wp_data

        if not dock_wp_data:
            self._slog.warn("dock_no_waypoint", action="skip_dock_resume_immediately")
            self._finish(True)
            return

        self._slog.info(
            "dock_start",
            x=dock_wp_data.get('pose', {}).get('x'),
            y=dock_wp_data.get('pose', {}).get('y'),
        )
        success = self._nav_mgr.send_goal(
            dock_wp_data,
            on_done=self._on_nav_to_dock_done,
            goal_id="dock_nav",
        )
        if not success:
            self._slog.error("dock_nav_send_failed")
            self._finish(False)

    def cancel(self):
        """Hủy docking (gọi khi operator stop)."""
        if self._charge_timer:
            self._charge_timer.cancel()
            self._node.destroy_timer(self._charge_timer)
            self._charge_timer = None
        self._nav_mgr.cancel()

    # ── Private ─────────────────────────────────────────────────────────────

    def _on_nav_to_dock_done(self, success: bool):
        if not success:
            self._slog.error("dock_nav_failed")
            self._finish(False)
            return

        self._slog.info("dock_arrived")
        # Register callback: BatteryManager gọi khi sạc xong
        self._battery_mgr._on_charged_cb = self._on_charge_complete

        # Heartbeat timer để log trạng thái sạc
        self._charge_timer = self._node.create_timer(
            self.CHARGE_CHECK_S, self._log_charging_status
        )

    def _log_charging_status(self):
        self._slog.info("charging_status", battery=self._battery_mgr.status_str())

    def _on_charge_complete(self):
        """Called by BatteryManager khi đủ pin để resume."""
        if self._charge_timer:
            self._charge_timer.cancel()
            self._node.destroy_timer(self._charge_timer)
            self._charge_timer = None
        self._slog.info("charge_complete")
        self._finish(True)

    def _finish(self, success: bool):
        self._battery_mgr.reset_flags()
        if self._on_complete:
            self._on_complete(success)
