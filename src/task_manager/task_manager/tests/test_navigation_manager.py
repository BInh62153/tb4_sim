"""
test_navigation_manager.py — Unit tests for NavigationManager.
Mocks ActionClient and service clients so no ROS2 spin needed.
"""

import math
import pytest
from unittest.mock import MagicMock, patch

from task_manager.managers.navigation_manager import NavigationManager


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


def make_nav_mgr():
    node = MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = MagicMock()
    node.create_client = MagicMock(return_value=MagicMock())
    slog = FakeLogger()

    with patch('task_manager.managers.navigation_manager.ActionClient') as MockAC:
        mock_client = MagicMock()
        MockAC.return_value = mock_client
        mgr = NavigationManager(node, slog)
        mgr._nav_client = mock_client
    return mgr, node


class TestSendGoal:
    def test_send_goal_when_server_not_ready(self):
        mgr, _ = make_nav_mgr()
        mgr._nav_client.server_is_ready.return_value = False
        cb = MagicMock()
        result = mgr.send_goal({'pose': {'x': 1.0, 'y': 0.0}}, cb)
        assert result is False
        cb.assert_not_called()

    def test_send_goal_when_server_ready(self):
        mgr, _ = make_nav_mgr()
        mgr._nav_client.server_is_ready.return_value = True
        mock_future = MagicMock()
        mgr._nav_client.send_goal_async.return_value = mock_future
        cb = MagicMock()
        result = mgr.send_goal({'pose': {'x': 1.0, 'y': 0.0, 'yaw': 0.5}}, cb)
        assert result is True
        mgr._nav_client.send_goal_async.assert_called_once()

    def test_goal_message_pose_fields(self):
        mgr, _ = make_nav_mgr()
        mgr._nav_client.server_is_ready.return_value = True
        mgr._nav_client.send_goal_async.return_value = MagicMock()

        sent_goals = []
        def capture(goal):
            sent_goals.append(goal)
            return MagicMock()
        mgr._nav_client.send_goal_async.side_effect = capture

        mgr.send_goal({'pose': {'x': 3.0, 'y': 4.0, 'yaw': 0.0}}, lambda _: None)
        g = sent_goals[0]
        assert g.pose.pose.position.x == 3.0
        assert g.pose.pose.position.y == 4.0
        assert g.pose.header.frame_id == 'map'


class TestGoalResponse:
    def test_rejected_goal_calls_on_done_false(self):
        mgr, _ = make_nav_mgr()
        cb = MagicMock()
        handle = MagicMock()
        handle.accepted = False
        future = MagicMock()
        future.result.return_value = handle
        mgr._on_goal_response(future, cb, "test")
        cb.assert_called_once_with(False)

    def test_accepted_goal_registers_result_callback(self):
        mgr, _ = make_nav_mgr()
        cb = MagicMock()
        handle = MagicMock()
        handle.accepted = True
        future = MagicMock()
        future.result.return_value = handle
        mgr._on_goal_response(future, cb, "test")
        handle.get_result_async.assert_called_once()


class TestCancel:
    def test_cancel_when_no_handle(self):
        mgr, _ = make_nav_mgr()
        mgr._current_goal_handle = None
        mgr.cancel()  # should not raise

    def test_cancel_calls_cancel_goal_async(self):
        mgr, _ = make_nav_mgr()
        handle = MagicMock()
        mgr._current_goal_handle = handle
        mgr.cancel()
        handle.cancel_goal_async.assert_called_once()
        assert mgr._current_goal_handle is None


class TestCostmapClear:
    def test_clear_both_costmaps_calls_both_services(self):
        mgr, _ = make_nav_mgr()

        futures = []
        callbacks = []

        def fake_call_async(req):
            f = MagicMock()
            def add_done_cb(cb):
                callbacks.append(cb)
            f.add_done_callback = add_done_cb
            futures.append(f)
            return f

        mgr._clear_local_srv.call_async  = fake_call_async
        mgr._clear_global_srv.call_async = fake_call_async

        done = MagicMock()
        mgr.clear_both_costmaps(done)

        assert len(callbacks) == 2
        done.assert_not_called()

        # Simulate both futures completing
        callbacks[0](None)
        done.assert_not_called()
        callbacks[1](None)
        done.assert_called_once()
