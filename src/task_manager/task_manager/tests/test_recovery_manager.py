"""
test_recovery_manager.py — Unit tests for 6-level RecoveryManager.
"""

import pytest
from unittest.mock import MagicMock, call, patch

from task_manager.managers.recovery_manager import RecoveryManager


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


def make_recovery():
    node = MagicMock()

    # create_timer returns a cancelable timer mock
    timer_mock = MagicMock()
    node.create_timer.return_value = timer_mock

    nav_mgr     = MagicMock()
    spin_client = MagicMock()
    slog        = FakeLogger()
    slog.recovery_level = 0

    with patch('task_manager.managers.recovery_manager.ActionClient') as MockAC:
        MockAC.return_value = MagicMock()
        rec = RecoveryManager(node, nav_mgr, spin_client, slog)

    return rec, node, nav_mgr, spin_client


class TestMaxAttempts:
    def test_abort_after_max_attempts(self):
        rec, _, _, _ = make_recovery()
        rec._attempt = RecoveryManager.MAX_ATTEMPTS
        cb = MagicMock()
        rec.start(cb)
        cb.assert_called_once_with(False)
        assert rec._attempt == 0

    def test_first_attempt_increments(self):
        rec, node, nav_mgr, _ = make_recovery()
        nav_mgr.clear_local_costmap = MagicMock()  # prevent further chaining
        rec.start(MagicMock())
        assert rec._attempt == 1

    def test_reset_clears_attempt(self):
        rec, _, _, _ = make_recovery()
        rec._attempt = 2
        rec.reset()
        assert rec._attempt == 0


class TestPipelineChaining:
    """Verify each level calls the next correctly."""

    def test_level1_creates_timer(self):
        rec, node, nav_mgr, _ = make_recovery()
        nav_mgr.clear_local_costmap = MagicMock()
        rec.start(MagicMock())
        node.create_timer.assert_called_once()

    def test_level2_calls_clear_local(self):
        rec, node, nav_mgr, _ = make_recovery()
        called = []
        nav_mgr.clear_local_costmap = lambda cb: called.append(('local', cb))
        # Skip wait: manually call level2
        rec._level2_clear_local(MagicMock())
        assert len(called) == 1

    def test_level3_calls_clear_global(self):
        rec, node, nav_mgr, _ = make_recovery()
        called = []
        nav_mgr.clear_global_costmap = lambda cb: called.append(('global', cb))
        rec._level3_clear_global(MagicMock())
        assert len(called) == 1

    def test_level4_spin_skipped_if_server_not_ready(self):
        rec, node, nav_mgr, spin_client = make_recovery()
        spin_client.wait_for_server.return_value = False
        nav_mgr.clear_global_costmap = MagicMock()  # level5 will call this
        # After spin is skipped, should call level5
        called = []
        rec._level5_backup = lambda cb: called.append('backup')
        rec._level4_spin(MagicMock())
        assert 'backup' in called

    def test_level6_replan_with_attempts_left(self):
        rec, _, _, _ = make_recovery()
        rec._attempt = 1  # < MAX_ATTEMPTS
        cb = MagicMock()
        rec._level6_replan(cb)
        cb.assert_called_once_with(True)

    def test_level6_abort_when_no_attempts_left(self):
        rec, _, _, _ = make_recovery()
        rec._attempt = RecoveryManager.MAX_ATTEMPTS
        cb = MagicMock()
        rec._level6_replan(cb)
        cb.assert_called_once_with(False)


class TestActionDoneHelper:
    def test_calls_next_on_accepted_goal(self):
        rec, _, _, _ = make_recovery()
        next_fn  = MagicMock()
        handle   = MagicMock()
        handle.accepted = True

        # Make get_result_async return a future that calls our callback
        result_future = MagicMock()
        captured_cb   = []
        def add_done_cb(cb):
            captured_cb.append(cb)
        result_future.add_done_callback = add_done_cb
        handle.get_result_async.return_value = result_future

        future = MagicMock()
        future.result.return_value = handle
        rec._on_action_done(future, next_fn)

        # next_fn called after result
        assert len(captured_cb) == 1
        captured_cb[0](None)
        next_fn.assert_called_once()

    def test_calls_next_if_exception(self):
        rec, _, _, _ = make_recovery()
        next_fn = MagicMock()
        future  = MagicMock()
        future.result.side_effect = Exception("boom")
        rec._on_action_done(future, next_fn)
        next_fn.assert_called_once()
