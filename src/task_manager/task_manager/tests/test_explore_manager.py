"""
test_explore_manager.py — Unit tests for ExploreManager.
Mocks subprocess.Popen + ROS publisher/service client, không cần ros2/colcon
hay tiến trình explore_lite thật.
"""

import pytest
from unittest.mock import MagicMock, patch

from task_manager.managers.explore_manager import (
    ExploreManager,
    ALGO_TO_CONTROLLER,
)


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


def make_running_proc():
    """subprocess.Popen mock: poll() trả None nghĩa là tiến trình còn sống."""
    proc = MagicMock()
    proc.poll.return_value = None
    return proc


def make_explore_mgr():
    node = MagicMock()
    resume_pub = MagicMock()
    set_param_client = MagicMock()
    set_param_client.wait_for_service.return_value = True
    set_param_client.call_async.return_value = MagicMock()

    node.create_publisher.return_value = resume_pub
    node.create_client.return_value = set_param_client

    slog = FakeLogger()
    em = ExploreManager(node, slog)
    return em, node, resume_pub, set_param_client


class TestIsRunning:
    def test_not_running_initially(self):
        em, *_ = make_explore_mgr()
        assert em.is_running is False
        assert em.is_exploring is False

    def test_running_when_process_alive(self):
        em, *_ = make_explore_mgr()
        em._proc = make_running_proc()
        assert em.is_running is True

    def test_not_running_when_process_exited(self):
        em, *_ = make_explore_mgr()
        proc = MagicMock()
        proc.poll.return_value = 0  # đã thoát
        em._proc = proc
        assert em.is_running is False


class TestStart:
    def test_start_spawns_process_when_not_running(self):
        em, *_ = make_explore_mgr()
        with patch('task_manager.managers.explore_manager.subprocess.Popen') as MockPopen:
            MockPopen.return_value = make_running_proc()
            em.start('dwa')
        MockPopen.assert_called_once()
        args = MockPopen.call_args[0][0]
        assert args[:3] == ['ros2', 'launch', 'explore_lite']

    def test_start_does_not_respawn_if_already_running(self):
        em, *_ = make_explore_mgr()
        em._proc = make_running_proc()
        with patch('task_manager.managers.explore_manager.subprocess.Popen') as MockPopen:
            em.start('teb')
        MockPopen.assert_not_called()

    def test_start_publishes_resume_true(self):
        em, _, resume_pub, _ = make_explore_mgr()
        em._proc = make_running_proc()
        em.start('dwa')
        resume_pub.publish.assert_called_once()
        published_msg = resume_pub.publish.call_args[0][0]
        assert published_msg.data is True

    def test_start_sets_exploring_flag(self):
        em, *_ = make_explore_mgr()
        em._proc = make_running_proc()
        em.start('pp')
        assert em.is_exploring is True
        assert em._current_algo == 'pp'

    def test_start_requests_controller_id_via_set_parameters(self):
        em, _, _, set_param_client = make_explore_mgr()
        em._proc = make_running_proc()
        em.start('teb')
        set_param_client.wait_for_service.assert_called_once()
        set_param_client.call_async.assert_called_once()
        req = set_param_client.call_async.call_args[0][0]
        assert req.parameters[0].name == 'controller_id'
        assert req.parameters[0].value.string_value == ALGO_TO_CONTROLLER['teb']

    def test_start_skips_set_param_when_service_unavailable(self):
        em, _, _, set_param_client = make_explore_mgr()
        set_param_client.wait_for_service.return_value = False
        em._proc = make_running_proc()
        em.start('teb')
        set_param_client.call_async.assert_not_called()


class TestPause:
    def test_pause_publishes_resume_false_when_running(self):
        em, _, resume_pub, _ = make_explore_mgr()
        em._proc = make_running_proc()
        em._exploring = True
        em.pause()
        published_msg = resume_pub.publish.call_args[0][0]
        assert published_msg.data is False
        assert em.is_exploring is False

    def test_pause_noop_when_not_running(self):
        em, _, resume_pub, _ = make_explore_mgr()
        em.pause()
        resume_pub.publish.assert_not_called()


class TestStop:
    def test_stop_terminates_process(self):
        em, *_ = make_explore_mgr()
        proc = make_running_proc()
        em._proc = proc
        em._exploring = True
        em.stop()
        proc.terminate.assert_called_once()
        assert em._proc is None
        assert em.is_exploring is False

    def test_stop_force_kills_on_timeout(self):
        em, *_ = make_explore_mgr()
        proc = make_running_proc()
        proc.wait.side_effect = Exception("timeout")
        em._proc = proc
        em.stop()
        proc.kill.assert_called_once()
        assert em._proc is None

    def test_stop_noop_when_already_stopped(self):
        em, *_ = make_explore_mgr()
        em.stop()  # không có proc -> không raise
        assert em.is_exploring is False


class TestSetAlgo:
    def test_set_algo_updates_controller_when_running(self):
        em, _, _, set_param_client = make_explore_mgr()
        em._proc = make_running_proc()
        em.set_algo('stanley')
        set_param_client.call_async.assert_called_once()
        assert em._current_algo == 'stanley'

    def test_set_algo_skips_param_call_when_not_running(self):
        em, _, _, set_param_client = make_explore_mgr()
        em.set_algo('stanley')
        set_param_client.call_async.assert_not_called()
        assert em._current_algo == 'stanley'  # vẫn nhớ lựa chọn cho lần start() sau


class TestOnSetParamDone:
    def test_logs_success_when_result_successful(self):
        em, *_ = make_explore_mgr()
        future = MagicMock()
        result_item = MagicMock(successful=True)
        future.result.return_value = MagicMock(results=[result_item])
        # Không raise là đủ để coi là pass (FakeLogger không assert nội dung)
        em._on_set_param_done(future, 'FollowPathTEB')

    def test_logs_warning_when_result_rejected(self):
        em, *_ = make_explore_mgr()
        future = MagicMock()
        result_item = MagicMock(successful=False, reason='invalid value')
        future.result.return_value = MagicMock(results=[result_item])
        em._on_set_param_done(future, 'FollowPathTEB')

    def test_handles_exception_from_future(self):
        em, *_ = make_explore_mgr()
        future = MagicMock()
        future.result.side_effect = Exception("service call failed")
        em._on_set_param_done(future, 'FollowPathTEB')  # không được raise