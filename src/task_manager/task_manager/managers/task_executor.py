#!/usr/bin/env python3
"""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TaskExecutor: chạy task sequence tại waypoint (wait, rotate, log, scan…).

Inject:
    node         – ROS2 LifecycleNode
    spin_client  – shared ActionClient[Spin]
    slog         – StructuredLogger
"""

from __future__ import annotations

import time
from typing import Callable, List, Optional

from nav2_msgs.action import Spin
from rclpy.action import ActionClient


class TaskExecutor:
    """Chạy async task list tại một waypoint."""

    def __init__(self, node, spin_client: ActionClient, slog):
        self._node        = node
        self._spin_client = spin_client  # shared
        self._slog        = slog

        self._tasks:       List[dict]    = []
        self._idx:         int           = 0
        self._on_complete: Optional[Callable] = None
        self._timer       = None
        self._start_time: Optional[float] = None
        # FIX: cancel() trước đây chỉ hủy timer 'wait', không hủy goal Spin
        # (task 'rotate') đang chạy -> 'stop'/'pause' trong lúc robot đang
        # quay (rotate task) không có tác dụng, robot vẫn quay hết vòng.
        self._spin_goal_handle = None
        # FIX: nếu không có cờ này, khi cancel() hủy 1 goal Spin đang chạy,
        # kết quả CANCELED của Spin vẫn về sau đó và _on_spin_result() sẽ
        # gọi _advance() -> tự động chạy tiếp task kế tiếp trong danh sách,
        # dù toàn bộ task list đáng lẽ phải dừng hẳn khi có lệnh stop/pause.
        self._cancelled = False

    # ── Public API ──────────────────────────────────────────────────────────

    def execute(self, tasks: List[dict], on_complete: Callable, waypoint: str = ""):
        """Bắt đầu chạy task list. Gọi on_complete() khi tất cả xong."""
        self._tasks      = tasks
        self._idx        = 0
        self._on_complete = on_complete
        self._cancelled   = False
        self._start_time = time.monotonic()
        self._slog.info(
            "tasks_start",
            task_count=len(tasks),
            waypoint=waypoint,
        )
        self._run_next()

    def cancel(self):
        self._cancelled = True
        if self._timer:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None
        if self._spin_goal_handle is not None:
            self._spin_goal_handle.cancel_goal_async()
            self._spin_goal_handle = None

    # ── Private ─────────────────────────────────────────────────────────────

    def _run_next(self):
        if self._idx >= len(self._tasks):
            elapsed = round(time.monotonic() - (self._start_time or 0), 2)
            self._slog.info("tasks_complete", task_duration_s=elapsed)
            if self._on_complete:
                self._on_complete()
            return

        task      = self._tasks[self._idx]
        task_type = task.get('type', 'log')
        self._slog.info(
            "task_run",
            task_idx=self._idx + 1,
            task_total=len(self._tasks),
            task_type=task_type,
        )

        if task_type == 'wait':
            self._do_wait(float(task.get('duration', 2.0)))

        elif task_type == 'rotate':
            self._do_rotate(float(task.get('angle', 1.5707)))

        elif task_type == 'log':
            self._node.get_logger().info(f"[Task] {task.get('message', '')}")
            self._advance()

        elif task_type == 'scan':
            # Placeholder: camera scan — advance ngay, có thể expand sau
            self._slog.info("task_scan", topic=task.get('topic', '?'))
            duration = float(task.get('duration', 1.0))
            self._do_wait(duration)

        else:
            self._slog.warn("task_unknown", task_type=task_type)
            self._advance()

    def _do_wait(self, duration: float):
        self._timer = self._node.create_timer(duration, self._on_timer_done)

    def _do_rotate(self, angle: float):
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self._slog.warn("task_spin_unavailable")
            self._advance()
            return
        goal            = Spin.Goal()
        goal.target_yaw = angle
        self._spin_client.send_goal_async(goal).add_done_callback(
            self._on_action_submitted
        )

    def _on_timer_done(self):
        if self._timer:
            self._timer.cancel()
            self._node.destroy_timer(self._timer)
            self._timer = None
        self._advance()

    def _on_action_submitted(self, future):
        handle = future.result()
        if not handle.accepted:
            self._advance()
            return
        self._spin_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, _future):
        self._spin_goal_handle = None
        if self._cancelled:
            return
        self._advance()

    def _advance(self):
        self._idx += 1
        self._run_next()