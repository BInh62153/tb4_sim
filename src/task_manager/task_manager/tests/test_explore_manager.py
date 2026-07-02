"""
test_explore_manager.py — Unit tests for ExploreManager.
Mocks subprocess.Popen + ROS publishers, không cần ros2/colcon
hay tiến trình explore_lite thật.

Cơ chế chọn controller: publish std_msgs/String lên topic
`/controller_selector` (cùng cơ chế NavigationManager dùng cho goto/patrol) —
KHÔNG còn dùng service `set_parameters` (đã bị xoá vì explore_node không hề
khai báo parameter `controller_id`, luôn bị reject).
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
    controller_pub = MagicMock()
    # Subscriber "đã sẵn sàng" ngay từ đầu -> tránh vòng chờ 0.5s trong test
    controller_pub.get_subscription_count.return_value = 1

    # ExploreManager tạo 2 publisher: /explore/resume rồi /controller_selector
    # (đúng thứ tự khai báo trong __init__) -> map theo thứ tự gọi.
    node.create_publisher.side_effect = [resume_pub, controller_pub]

    slog = FakeLogger()
    em = ExploreManager(node, slog)
    return em, node, resume_pub, controller_pub


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

    def test_start_publishes_controller_id_via_controller_selector_topic(self):
        em, _, _, controller_pub = make_explore_mgr()
        em._proc = make_running_proc()
        em.start('teb')
        controller_pub.publish.assert_called_once()
        published_msg = controller_pub.publish.call_args[0][0]
        assert published_msg.data == ALGO_TO_CONTROLLER['teb']

    def test_start_waits_for_subscriber_before_publishing(self):
        em, _, _, controller_pub = make_explore_mgr()
        # Chưa có subscriber lúc đầu, xuất hiện sau vài lần poll
        controller_pub.get_subscription_count.side_effect = [0, 0, 1]
        em._proc = make_running_proc()
        with patch('task_manager.managers.explore_manager.time.sleep'):
            em.start('dwa')
        controller_pub.publish.assert_called_once()


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
        em, _, _, controller_pub = make_explore_mgr()
        em._proc = make_running_proc()
        em.set_algo('stanley')
        controller_pub.publish.assert_called_once()
        published_msg = controller_pub.publish.call_args[0][0]
        # 'stanley' chưa có trong ALGO_TO_CONTROLLER -> fallback FollowPathDWA
        assert published_msg.data == 'FollowPathDWA'
        assert em._current_algo == 'stanley'

    def test_set_algo_skips_publish_when_not_running(self):
        em, _, _, controller_pub = make_explore_mgr()
        em.set_algo('teb')
        controller_pub.publish.assert_not_called()
        assert em._current_algo == 'teb'  # vẫn nhớ lựa chọn cho lần start() sau