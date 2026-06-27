"""
test_structured_logger.py — Unit tests for StructuredLogger.
"""

import pytest
from task_manager.state.structured_logger import StructuredLogger


class Capture:
    def __init__(self):
        self.lines = []
    def info(self, m):  self.lines.append(('I', m))
    def warn(self, m):  self.lines.append(('W', m))
    def error(self, m): self.lines.append(('E', m))
    def debug(self, m): self.lines.append(('D', m))


@pytest.fixture
def slog():
    cap = Capture()
    sl  = StructuredLogger(cap, mission_id="test01")
    sl._raw = cap  # expose for assertions
    return sl


class TestFormat:
    def test_includes_mission_id(self, slog):
        slog.info("some_event")
        assert "mission=test01" in slog._raw.lines[-1][1]

    def test_includes_event_name(self, slog):
        slog.info("nav_goal_sent")
        assert "event=nav_goal_sent" in slog._raw.lines[-1][1]

    def test_includes_extra_kwargs(self, slog):
        slog.info("nav_goal_sent", x=1.5, y=2.0)
        line = slog._raw.lines[-1][1]
        assert "x=1.5" in line
        assert "y=2.0" in line

    def test_goal_id_included_when_set(self, slog):
        slog.goal_id = "abc123"
        slog.info("nav_start")
        assert "goal=abc123" in slog._raw.lines[-1][1]

    def test_goal_id_omitted_when_empty(self, slog):
        slog.goal_id = ""
        slog.info("nav_start")
        assert "goal=" not in slog._raw.lines[-1][1]

    def test_retry_count_shown(self, slog):
        slog.retry_count = 2
        slog.warn("retry")
        assert "retry=2" in slog._raw.lines[-1][1]

    def test_recovery_level_shown(self, slog):
        slog.recovery_level = 4
        slog.warn("spin")
        assert "recovery_lvl=4" in slog._raw.lines[-1][1]

    def test_level_prefix_struct(self, slog):
        slog.info("test")
        assert slog._raw.lines[-1][1].startswith("[STRUCT]")


class TestLevels:
    def test_warn_uses_warn_channel(self, slog):
        slog.warn("low_bat")
        assert slog._raw.lines[-1][0] == 'W'

    def test_error_uses_error_channel(self, slog):
        slog.error("boom")
        assert slog._raw.lines[-1][0] == 'E'

    def test_debug_uses_debug_channel(self, slog):
        slog.debug("verbose")
        assert slog._raw.lines[-1][0] == 'D'
