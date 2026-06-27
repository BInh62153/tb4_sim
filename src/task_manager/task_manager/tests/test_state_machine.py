"""
test_state_machine.py — Unit tests for StateMachine + Event dispatch.
"""

import pytest
from task_manager.state.states import SystemState
from task_manager.state.state_machine import StateMachine, Event


class _Log:
    def __init__(self):
        self.msgs = []
    def info(self, m):  self.msgs.append(('I', m))
    def warn(self, m):  self.msgs.append(('W', m))
    def error(self, m): self.msgs.append(('E', m))
    def debug(self, m): pass


@pytest.fixture
def sm():
    return StateMachine(_Log())


class TestInitialState:
    def test_starts_unconfigured(self, sm):
        assert sm.state == SystemState.UNCONFIGURED

    def test_is_in(self, sm):
        assert sm.is_in(SystemState.UNCONFIGURED)
        assert not sm.is_in(SystemState.IDLE)


class TestValidTransitions:
    def test_unconfigured_to_idle(self, sm):
        assert sm.transition(SystemState.IDLE) is True
        assert sm.state == SystemState.IDLE

    def test_idle_to_navigating(self, sm):
        sm.transition(SystemState.IDLE)
        assert sm.transition(SystemState.NAVIGATING) is True

    def test_navigating_to_executing(self, sm):
        sm.transition(SystemState.IDLE)
        sm.transition(SystemState.NAVIGATING)
        assert sm.transition(SystemState.EXECUTING_TASK) is True

    def test_navigating_to_recovering(self, sm):
        sm.transition(SystemState.IDLE)
        sm.transition(SystemState.NAVIGATING)
        assert sm.transition(SystemState.RECOVERING) is True

    def test_recovering_to_idle(self, sm):
        sm.transition(SystemState.IDLE)
        sm.transition(SystemState.NAVIGATING)
        sm.transition(SystemState.RECOVERING)
        assert sm.transition(SystemState.IDLE) is True


class TestInvalidTransitions:
    def test_unconfigured_cannot_go_navigating(self, sm):
        result = sm.transition(SystemState.NAVIGATING)
        assert result is False
        assert sm.state == SystemState.UNCONFIGURED  # unchanged

    def test_idle_cannot_go_charging_directly(self, sm):
        sm.transition(SystemState.IDLE)
        result = sm.transition(SystemState.CHARGING)
        assert result is False


class TestForceTransition:
    def test_force_bypasses_validation(self, sm):
        sm.force_transition(SystemState.NAVIGATING, "test")
        assert sm.state == SystemState.NAVIGATING


class TestEventDispatch:
    def test_register_and_dispatch(self, sm):
        calls = []
        sm.register(Event.NAV_SUCCESS, lambda **kw: calls.append(kw))
        sm.dispatch(Event.NAV_SUCCESS, goal_id="abc")
        assert len(calls) == 1
        assert calls[0]['goal_id'] == 'abc'

    def test_multiple_handlers(self, sm):
        results = []
        sm.register(Event.NAV_FAILED, lambda **_: results.append(1))
        sm.register(Event.NAV_FAILED, lambda **_: results.append(2))
        sm.dispatch(Event.NAV_FAILED)
        assert results == [1, 2]

    def test_dispatch_no_handlers_is_safe(self, sm):
        # Should not raise
        sm.dispatch(Event.TASKS_COMPLETE)

    def test_handler_exception_does_not_propagate(self, sm):
        def bad(**_):
            raise RuntimeError("boom")
        sm.register(Event.RECOVERY_ABORTED, bad)
        sm.dispatch(Event.RECOVERY_ABORTED)  # should not raise
