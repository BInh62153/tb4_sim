"""
test_explore_manager.py — Unit tests for ExploreManager.
Mocks subprocess.Popen + ROS publishers, không cần ros2/colcon
hay tiến trình explore_lite thật.

Cơ chế chọn controller: publish std_msgs/String lên topic
`/controller_selector` (cùng cơ chế NavigationManager dùng cho goto/patrol) —
KHÔNG còn dùng service `set_parameters` (đã bị xoá vì explore_node không hề
khai báo parameter `controller_id`, luôn bị reject).

Cơ chế stop(): gửi SIGTERM cho cả process group qua os.killpg() ngay lập tức
(không block), rồi lên lịch một timer 1.5s gọi _force_kill_if_alive() để
SIGKILL cưỡng bức nếu tiến trình vẫn còn sống — thay cho proc.wait(timeout=...)
+ proc.terminate()/proc.kill() cũ (từng block executor đơn luồng).
"""

import signal

import pytest
from unittest.mock import MagicMock, patch

from task_manager.managers.explore_manager import (
    ExploreManager,
    ALGO_TO_CONTROLLER,
    _SELECTOR_QOS,
)


class FakeLogger:
    def info(self, m="", **kw): pass
    def warn(self, m="", **kw): pass
    def error(self, m="", **kw): pass
    def debug(self, m="", **kw): pass


def make_running_proc(pid=1234):
    """subprocess.Popen mock: poll() trả None nghĩa là tiến trình còn sống."""
    proc = MagicMock()
    proc.poll.return_value = None
    proc.pid = pid
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

    def test_controller_selector_publisher_uses_latched_qos(self):
        # Thay cho test_start_waits_for_subscriber_before_publishing (cũ):
        # không còn poll get_subscription_count()/time.sleep() nữa — publisher
        # /controller_selector được tạo với QoS TRANSIENT_LOCAL (latched) ngay
        # từ __init__, nên late-joining subscriber vẫn nhận được value cuối
        # mà không cần chờ.
        em, node, _, _ = make_explore_mgr()
        calls = node.create_publisher.call_args_list
        # thứ tự tạo publisher trong __init__: /explore/resume rồi /controller_selector
        topic_arg = calls[1][0][1]
        qos_arg = calls[1][0][2]
        assert topic_arg == '/controller_selector'
        assert qos_arg is _SELECTOR_QOS
        assert _SELECTOR_QOS.durability.name == 'TRANSIENT_LOCAL' \
            if hasattr(_SELECTOR_QOS.durability, 'name') else True


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
    def test_stop_sends_sigterm_to_process_group_and_schedules_grace_timer(self):
        em, *_ = make_explore_mgr()
        proc = make_running_proc(pid=1234)
        em._proc = proc
        em._exploring = True

        with patch('task_manager.managers.explore_manager.os.getpgid', return_value=1234) as mock_getpgid, \
             patch('task_manager.managers.explore_manager.os.killpg') as mock_killpg:
            em.stop()

        mock_getpgid.assert_called_once_with(1234)
        mock_killpg.assert_called_once_with(1234, signal.SIGTERM)
        # state đã cập nhật ngay lập tức, không đợi tiến trình thật sự thoát
        assert em._proc is None
        assert em.is_exploring is False
        # timer cưỡng bức kill sau grace period phải được lên lịch (1.5s)
        em._node.create_timer.assert_called_once()
        assert em._node.create_timer.call_args[0][0] == 1.5

    def test_force_kill_if_alive_sends_sigkill_when_process_still_running(self):
        em, *_ = make_explore_mgr()
        proc = make_running_proc(pid=1234)
        em._proc = proc

        with patch('task_manager.managers.explore_manager.os.getpgid', return_value=1234), \
             patch('task_manager.managers.explore_manager.os.killpg'):
            em.stop()

        # mô phỏng timer 1.5s hết hạn trong khi tiến trình vẫn còn sống
        # (proc.poll() vẫn trả None, xem make_running_proc)
        with patch('task_manager.managers.explore_manager.os.getpgid', return_value=1234) as mock_getpgid, \
             patch('task_manager.managers.explore_manager.os.killpg') as mock_killpg:
            em._force_kill_if_alive()

        mock_getpgid.assert_called_once_with(1234)
        mock_killpg.assert_called_once_with(1234, signal.SIGKILL)

    def test_force_kill_if_alive_skips_when_process_already_exited(self):
        em, *_ = make_explore_mgr()
        proc = make_running_proc(pid=1234)
        em._proc = proc

        with patch('task_manager.managers.explore_manager.os.getpgid', return_value=1234), \
             patch('task_manager.managers.explore_manager.os.killpg'):
            em.stop()

        # tiến trình đã tự thoát trước khi timer kịp chạy
        em._pending_kill_proc.poll.return_value = 0

        with patch('task_manager.managers.explore_manager.os.killpg') as mock_killpg:
            em._force_kill_if_alive()

        mock_killpg.assert_not_called()

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