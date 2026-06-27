"""
test_dock_manager.py — Unit tests for DockManager.
"""

import pytest
from unittest.mock import MagicMock

from task_manager.managers.dock_manager import DockManager


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


def make_dock():
    node = MagicMock()
    timer_mock = MagicMock()
    node.create_timer.return_value = timer_mock
    nav_mgr     = MagicMock()
    battery_mgr = MagicMock()
    battery_mgr.status_str.return_value = "50%"
    slog        = FakeLogger()
    dm = DockManager(node, nav_mgr, battery_mgr, slog)
    return dm, node, nav_mgr, battery_mgr


class TestNoDockWaypoint:
    def test_no_waypoint_data_calls_complete_true(self):
        dm, _, _, _ = make_dock()
        cb = MagicMock()
        dm.start_docking(None, cb)
        cb.assert_called_once_with(True)


class TestNavToDock:
    def test_nav_send_success_waits(self):
        dm, node, nav_mgr, battery_mgr = make_dock()
        nav_mgr.send_goal.return_value = True
        cb = MagicMock()
        dock_wp = {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'tasks': []}
        dm.start_docking(dock_wp, cb)
        nav_mgr.send_goal.assert_called_once()
        cb.assert_not_called()  # waiting for dock

    def test_nav_send_failure_calls_complete_false(self):
        dm, _, nav_mgr, _ = make_dock()
        nav_mgr.send_goal.return_value = False
        cb = MagicMock()
        dm.start_docking({'pose': {}}, cb)
        cb.assert_called_once_with(False)


class TestNavToDockResult:
    def test_nav_failed_calls_complete_false(self):
        dm, _, nav_mgr, _ = make_dock()
        nav_mgr.send_goal.return_value = True
        cb = MagicMock()
        dm.start_docking({'pose': {}}, cb)
        dm._on_nav_to_dock_done(False)
        cb.assert_called_once_with(False)

    def test_nav_success_starts_charge_timer(self):
        dm, node, nav_mgr, battery_mgr = make_dock()
        nav_mgr.send_goal.return_value = True
        dm.start_docking({'pose': {}}, MagicMock())
        dm._on_nav_to_dock_done(True)
        node.create_timer.assert_called()

    def test_charge_complete_fires_on_complete_true(self):
        dm, _, nav_mgr, battery_mgr = make_dock()
        nav_mgr.send_goal.return_value = True
        cb = MagicMock()
        dm.start_docking({'pose': {}}, cb)
        dm._on_nav_to_dock_done(True)
        dm._on_charge_complete()
        cb.assert_called_once_with(True)

    def test_charge_complete_resets_battery_flags(self):
        dm, _, nav_mgr, battery_mgr = make_dock()
        nav_mgr.send_goal.return_value = True
        dm.start_docking({'pose': {}}, MagicMock())
        dm._on_nav_to_dock_done(True)
        dm._on_charge_complete()
        battery_mgr.reset_flags.assert_called()


class TestCancel:
    def test_cancel_calls_nav_cancel(self):
        dm, _, nav_mgr, _ = make_dock()
        dm.cancel()
        nav_mgr.cancel.assert_called_once()
