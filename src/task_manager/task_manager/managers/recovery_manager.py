#!/usr/bin/env python3
"""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
RecoveryManager: Nav2-standard 6-level recovery tree.

Level 1 → wait
Level 2 → clear local costmap
Level 3 → clear global costmap
Level 4 → spin 90°
Level 5 → backup 0.2 m
Level 6 → replan (retry nav) → abort nếu vẫn fail

Inject:
    node         – ROS2 LifecycleNode
    nav_mgr      – NavigationManager (costmap clears)
    spin_client  – shared ActionClient[Spin]
    slog         – StructuredLogger
"""

from __future__ import annotations

import math
from typing import Callable, Optional

from nav2_msgs.action import BackUp, Spin
from rclpy.action import ActionClient


class RecoveryManager:
    """6-level Nav2 recovery pipeline."""

    MAX_ATTEMPTS = 3  # số lần thử toàn bộ pipeline trước khi abort

    def __init__(self, node, nav_mgr, spin_client: ActionClient, slog):
        self._node         = node
        self._nav_mgr      = nav_mgr
        self._spin_client  = spin_client  # shared
        self._backup_client = ActionClient(node, BackUp, 'backup')
        self._slog         = slog
        self._attempt      = 0
        self._wait_timer   = None

    # ── Public API ──────────────────────────────────────────────────────────

    def start(self, on_complete: Callable[[bool], None]):
        """
        Bắt đầu recovery pipeline.
        on_complete(success) được gọi khi xong.
        """
        if self._attempt >= self.MAX_ATTEMPTS:
            self._slog.error("recovery_abort", attempt=self._attempt)
            self._attempt = 0
            on_complete(False)
            return

        self._attempt += 1
        self._slog.warn(
            "recovery_start",
            recovery_level=1,
            attempt=self._attempt,
            max=self.MAX_ATTEMPTS,
        )
        self._level1_wait(on_complete)

    def reset(self):
        self._attempt = 0

    @property
    def attempt(self) -> int:
        return self._attempt

    # ── Level 1: Wait ───────────────────────────────────────────────────────

    def _level1_wait(self, cb):
        self._slog.warn("recovery_level", level=1, action="wait_2s")
        self._wait_timer = self._node.create_timer(
            2.0,
            lambda: self._on_wait_done(cb),
        )

    def _on_wait_done(self, cb):
        if self._wait_timer:
            self._wait_timer.cancel()
            self._node.destroy_timer(self._wait_timer)
            self._wait_timer = None
        self._level2_clear_local(cb)

    # ── Level 2: Clear local costmap ────────────────────────────────────────

    def _level2_clear_local(self, cb):
        self._slog.warn("recovery_level", level=2, action="clear_local_costmap")
        self._nav_mgr.clear_local_costmap(
            lambda: self._level3_clear_global(cb)
        )

    # ── Level 3: Clear global costmap ───────────────────────────────────────

    def _level3_clear_global(self, cb):
        self._slog.warn("recovery_level", level=3, action="clear_global_costmap")
        self._nav_mgr.clear_global_costmap(
            lambda: self._level4_spin(cb)
        )

    # ── Level 4: Spin 90° ───────────────────────────────────────────────────

    def _level4_spin(self, cb):
        self._slog.warn("recovery_level", level=4, action="spin_90deg")
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self._slog.warn("recovery_skip_spin", reason="server_not_ready")
            self._level5_backup(cb)
            return

        goal            = Spin.Goal()
        goal.target_yaw = math.pi / 2
        self._spin_client.send_goal_async(goal).add_done_callback(
            lambda f: self._on_action_done(f, lambda: self._level5_backup(cb))
        )

    # ── Level 5: Backup ─────────────────────────────────────────────────────

    def _level5_backup(self, cb):
        self._slog.warn("recovery_level", level=5, action="backup_0.2m")
        if not self._backup_client.wait_for_server(timeout_sec=2.0):
            self._slog.warn("recovery_skip_backup", reason="server_not_ready")
            self._level6_replan(cb)
            return

        goal          = BackUp.Goal()
        goal.target.x = -0.2
        goal.speed    = 0.05
        self._backup_client.send_goal_async(goal).add_done_callback(
            lambda f: self._on_action_done(f, lambda: self._level6_replan(cb))
        )

    # ── Level 6: Replan / Abort ─────────────────────────────────────────────

    def _level6_replan(self, cb):
        self._slog.warn("recovery_level", level=6, action="replan_or_abort")
        # Replan = trả control về NavigationManager để retry nav goal
        # Nếu attempt còn → cb(True) → node retry nav
        # Nếu attempt đã hết → cb(False) → abort waypoint
        if self._attempt < self.MAX_ATTEMPTS:
            self._slog.info("recovery_replan", will_retry=True)
            cb(True)
        else:
            self._slog.error("recovery_abort_final")
            cb(False)

    # ── Helper ──────────────────────────────────────────────────────────────

    def _on_action_done(self, future, next_fn: Callable):
        """Gọi next_fn sau khi action xong, bỏ qua lỗi."""
        try:
            handle = future.result()
            if handle and handle.accepted:
                handle.get_result_async().add_done_callback(lambda _: next_fn())
                return
        except Exception:
            pass
        next_fn()
