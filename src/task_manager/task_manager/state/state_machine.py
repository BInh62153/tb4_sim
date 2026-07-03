#!/usr/bin/env python3
"""
StateMachine tập trung: tất cả transition đi qua đây.

  sm = StateMachine(logger)
  sm.transition(SystemState.NAVIGATING)      # validate + log
  sm.dispatch(Event.NAV_SUCCESS)             # fire registered handlers
  sm.register(Event.NAV_SUCCESS, callback)   # đăng ký handler
"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Dict, List, Optional

from .states import SystemState, VALID_TRANSITIONS


class Event(Enum):
    """Tất cả sự kiện nội bộ trong hệ thống."""
    # Navigation
    NAV_GOAL_SENT       = auto()
    NAV_SUCCESS         = auto()
    NAV_FAILED          = auto()

    # Task execution
    TASKS_COMPLETE      = auto()
    TASK_FAILED         = auto()

    # Recovery
    RECOVERY_SUCCESS    = auto()
    RECOVERY_ABORTED    = auto()

    # Battery / Dock
    BATTERY_LOW         = auto()
    BATTERY_CRITICAL    = auto()
    DOCK_ARRIVED        = auto()
    DOCK_COMPLETE       = auto()
    DOCK_FAILED         = auto()
    CHARGE_COMPLETE     = auto()

    # Operator commands
    CMD_PATROL          = auto()
    CMD_PAUSE           = auto()
    CMD_STOP            = auto()
    CMD_GOTO            = auto()
    CMD_GOTO_POS        = auto()
    CMD_RESUME          = auto()
    CMD_EXPLORE         = auto()


class StateMachine:
    """
    Centralized finite state machine.

    - transition(): validate + ghi log + cập nhật state.
    - dispatch(): fire tất cả handler đã đăng ký cho event đó.
    - register(): đăng ký callback cho một Event.
    """

    def __init__(self, logger, structured_log: Optional[Callable] = None):
        self._state    = SystemState.UNCONFIGURED
        self._logger   = logger
        self._slog     = structured_log  # callback for structured logs
        self._handlers: Dict[Event, List[Callable]] = {e: [] for e in Event}

    # ── Public API ─────────────────────────────────────────────────────────

    @property
    def state(self) -> SystemState:
        return self._state

    def transition(self, new_state: SystemState, reason: str = "") -> bool:
        """
        Chuyển state.
        Trả về False và log WARNING nếu transition không hợp lệ.
        """
        allowed = VALID_TRANSITIONS.get(self._state, set())
        if new_state not in allowed:
            self._logger.warn(
                f"[SM] INVALID transition {self._state.name} → {new_state.name} "
                f"(reason: {reason or 'none'})"
            )
            return False

        self._logger.info(
            f"[SM] {self._state.name} → {new_state.name}"
            + (f" [{reason}]" if reason else "")
        )
        self._state = new_state
        return True

    def force_transition(self, new_state: SystemState, reason: str = "FORCED"):
        """
        Bypass validation — chỉ dùng trong lifecycle callbacks (on_cleanup, on_error).
        """
        self._logger.warn(
            f"[SM] FORCE {self._state.name} → {new_state.name} [{reason}]"
        )
        self._state = new_state

    def dispatch(self, event: Event, **kwargs):
        """Gọi tất cả handler đã đăng ký cho event này."""
        self._logger.debug(f"[SM] Event: {event.name} kwargs={list(kwargs.keys())}")
        for handler in self._handlers.get(event, []):
            try:
                handler(**kwargs)
            except Exception as exc:
                self._logger.error(
                    f"[SM] Handler {handler.__name__!r} raised: {exc}"
                )

    def register(self, event: Event, callback: Callable):
        """Đăng ký callback cho event."""
        self._handlers[event].append(callback)

    def is_in(self, *states: SystemState) -> bool:
        return self._state in states