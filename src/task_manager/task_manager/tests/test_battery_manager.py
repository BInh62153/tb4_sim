"""
test_battery_manager.py — Unit tests for BatteryManager.
No ROS2 required — we mock the node and subscription.
"""

import pytest
from unittest.mock import MagicMock, patch

from task_manager.managers.battery_manager import BatteryManager


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


class FakeBatteryState:
    POWER_SUPPLY_STATUS_CHARGING     = 1
    POWER_SUPPLY_STATUS_DISCHARGING  = 2

    def __init__(self, pct_raw, status):
        self.percentage          = pct_raw   # 0.0–1.0
        self.power_supply_status = status


def make_mgr(params=None) -> BatteryManager:
    node = MagicMock()
    node.create_subscription = MagicMock(return_value=MagicMock())
    slog = FakeLogger()
    mgr  = BatteryManager(node, slog, params or {})
    return mgr


class TestThresholds:
    def test_default_thresholds(self):
        mgr = make_mgr()
        assert mgr.low_pct      == 20.0
        assert mgr.resume_pct   == 80.0
        assert mgr.critical_pct == 10.0

    def test_custom_thresholds(self):
        mgr = make_mgr({'low_battery_pct': 30.0, 'resume_battery_pct': 90.0})
        assert mgr.low_pct    == 30.0
        assert mgr.resume_pct == 90.0


class TestCallbacks:
    def _make_started(self, params=None):
        mgr = make_mgr(params)
        low_cb      = MagicMock()
        crit_cb     = MagicMock()
        charged_cb  = MagicMock()
        mgr.start(on_low=low_cb, on_critical=crit_cb, on_charged=charged_cb)
        return mgr, low_cb, crit_cb, charged_cb

    def test_low_battery_fires(self):
        mgr, low_cb, crit_cb, _ = self._make_started()
        msg = FakeBatteryState(0.15, FakeBatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        mgr._cb(msg)
        low_cb.assert_called_once()
        crit_cb.assert_not_called()
        assert mgr.is_low is True

    def test_critical_battery_fires(self):
        mgr, _, crit_cb, _ = self._make_started()
        msg = FakeBatteryState(0.05, FakeBatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        mgr._cb(msg)
        crit_cb.assert_called_once()
        assert mgr.is_critical is True

    def test_low_cb_not_fired_twice(self):
        mgr, low_cb, _, _ = self._make_started()
        msg = FakeBatteryState(0.15, FakeBatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        mgr._cb(msg)
        mgr._cb(msg)
        assert low_cb.call_count == 1

    def test_charged_fires_when_charging_above_resume(self):
        mgr, _, _, charged_cb = self._make_started()
        # First make it low
        low_msg = FakeBatteryState(0.15, FakeBatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        mgr._cb(low_msg)
        assert mgr.is_low is True
        # Now charging above resume threshold
        charged_msg = FakeBatteryState(0.85, FakeBatteryState.POWER_SUPPLY_STATUS_CHARGING)
        mgr._cb(charged_msg)
        charged_cb.assert_called_once()
        assert mgr.is_low is False

    def test_charging_prevents_low_trigger(self):
        mgr, low_cb, _, _ = self._make_started()
        msg = FakeBatteryState(0.15, FakeBatteryState.POWER_SUPPLY_STATUS_CHARGING)
        mgr._cb(msg)
        low_cb.assert_not_called()

    def test_no_callbacks_if_none_registered(self):
        node = MagicMock()
        node.create_subscription = MagicMock(return_value=MagicMock())
        mgr  = BatteryManager(node, FakeLogger())
        mgr.start()  # no callbacks
        msg = FakeBatteryState(0.05, FakeBatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        mgr._cb(msg)  # should not raise


class TestStatusStr:
    def test_status_str_includes_pct(self):
        mgr = make_mgr()
        mgr.percentage = 42.0
        s = mgr.status_str()
        assert '42.0%' in s

    def test_status_str_shows_low(self):
        mgr = make_mgr()
        mgr.percentage = 15.0
        mgr.is_low     = True
        assert '[LOW]' in mgr.status_str()
