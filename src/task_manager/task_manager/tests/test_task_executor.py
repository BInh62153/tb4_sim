"""
test_task_executor.py — Unit tests for TaskExecutor.
"""

import pytest
from unittest.mock import MagicMock, patch

from task_manager.managers.task_executor import TaskExecutor


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


def make_executor():
    node = MagicMock()
    timer_mock = MagicMock()
    node.create_timer.return_value = timer_mock

    spin_client = MagicMock()
    slog = FakeLogger()

    ex = TaskExecutor(node, spin_client, slog)
    return ex, node, spin_client


class TestEmptyTasks:
    def test_empty_task_list_calls_on_complete(self):
        ex, _, _ = make_executor()
        cb = MagicMock()
        ex.execute([], cb)
        cb.assert_called_once()


class TestLogTask:
    def test_log_task_advances_and_completes(self):
        ex, _, _ = make_executor()
        cb = MagicMock()
        ex.execute([{'type': 'log', 'message': 'hello'}], cb)
        cb.assert_called_once()


class TestWaitTask:
    def test_wait_creates_timer(self):
        ex, node, _ = make_executor()
        cb = MagicMock()
        ex.execute([{'type': 'wait', 'duration': 3.0}], cb)
        node.create_timer.assert_called_with(3.0, ex._on_timer_done)

    def test_timer_done_advances(self):
        ex, node, _ = make_executor()
        cb = MagicMock()
        ex.execute([{'type': 'wait', 'duration': 1.0}], cb)
        ex._on_timer_done()
        cb.assert_called_once()


class TestRotateTask:
    def test_rotate_skipped_when_server_not_ready(self):
        ex, _, spin_client = make_executor()
        spin_client.wait_for_server.return_value = False
        cb = MagicMock()
        ex.execute([{'type': 'rotate', 'angle': 1.5707}], cb)
        cb.assert_called_once()  # skip → complete

    def test_rotate_sends_goal(self):
        ex, _, spin_client = make_executor()
        spin_client.wait_for_server.return_value = True
        future_mock = MagicMock()
        spin_client.send_goal_async.return_value = future_mock
        cb = MagicMock()
        ex.execute([{'type': 'rotate', 'angle': 1.5707}], cb)
        spin_client.send_goal_async.assert_called_once()


class TestUnknownTask:
    def test_unknown_task_skips_and_continues(self):
        ex, _, _ = make_executor()
        cb = MagicMock()
        tasks = [
            {'type': 'teleport'},   # unknown
            {'type': 'log', 'message': 'after'},
        ]
        ex.execute(tasks, cb)
        cb.assert_called_once()


class TestMultiTaskSequence:
    def test_multiple_log_tasks_all_run(self):
        ex, _, _ = make_executor()
        results = []
        tasks = [
            {'type': 'log', 'message': 'step1'},
            {'type': 'log', 'message': 'step2'},
            {'type': 'log', 'message': 'step3'},
        ]
        ex.execute(tasks, lambda: results.append('done'))
        assert results == ['done']


class TestCancel:
    def test_cancel_destroys_timer(self):
        ex, node, _ = make_executor()
        ex.execute([{'type': 'wait', 'duration': 5.0}], MagicMock())
        ex.cancel()
        node.destroy_timer.assert_called()
