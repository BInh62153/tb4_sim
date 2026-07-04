#!/usr/bin/env python3
"""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ExploreManager: khởi động/dừng node `explore_lite` (package m-explore-ros2)
và đồng bộ lựa chọn thuật toán (algo) với controller_id của nó — cùng cách
NavigationManager làm cho goto/patrol.

explore_lite là 1 node C++ độc lập (không chạy trong tiến trình Python của
task_manager), nhưng nó gửi goal tới BT Navigator qua action `navigate_to_pose`
giống hệt NavigationManager — nghĩa là node `ControllerSelector` trong
navigate_to_pose.xml/navigate_through_poses.xml (đọc từ topic
`/controller_selector`) áp dụng cho MỌI client gửi goal, không riêng gì
NavigationManager. Vì vậy không cần patch C++ / thêm parameter tùy biến trên
explore_node — chỉ cần publish cùng topic `/controller_selector` trước khi
explore_lite gửi goal tiếp theo là đủ.

  • start/stop tiến trình  → spawn/kill qua `ros2 launch explore_lite
    explore.launch.py` (subprocess).
  • pause/resume tìm frontier (không kill tiến trình) → publish
    std_msgs/Bool lên topic `/explore/resume` (đã có sẵn trong explore.cpp,
    xem Explore::resumeCallback — không cần sửa).
  • chọn thuật toán lái (dwa/teb/pp/stanley) → publish std_msgs/String lên
    topic `/controller_selector` (BT ControllerSelector đọc, set blackboard
    `selected_controller`, FollowPath node dùng giá trị đó) — CÙNG cơ chế
    NavigationManager đang dùng, không phải service set_parameters (node
    explore_node không hề khai báo parameter `controller_id`, gọi sẽ luôn
    bị reject).

Inject:
    node  – ROS2 LifecycleNode (để tạo publisher/service client)
    slog  – StructuredLogger
"""

from __future__ import annotations

import os
import signal
import subprocess
from typing import Optional

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

#: QoS "latched" — xem giải thích chi tiết trong navigation_manager.py.
#: Dùng chung định nghĩa để publisher /controller_selector nhất quán dù được
#: tạo từ NavigationManager hay ExploreManager.
_SELECTOR_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


#: Map tên thuật toán sang controller_id (giống ALGO_TO_CONTROLLER trong
#: navigation_manager.py — giữ 1 nguồn sự thật sẽ tốt hơn nếu sau này gộp
#: 2 module lại, tạm thời lặp lại cho ExploreManager độc lập với Nav2 action).
ALGO_TO_CONTROLLER = {
    'dwa':     'FollowPathDWA',
    'teb':     'FollowPathTEB',
    'pp':      'FollowPathPP',
    # 'stanley': 'FollowPathStanley',  # chưa implement — xem TODO trong nav2_params.yaml
}


class ExploreManager:
    """Điều phối vòng đời + thuật toán của node explore_lite."""

    def __init__(self, node, slog):
        self._node = node
        self._slog = slog

        self._proc: Optional[subprocess.Popen] = None
        self._exploring = False   # đang active tìm frontier (chưa pause)
        self._current_algo = 'dwa'
        # FIX: tiến trình chờ SIGKILL sau grace period (nếu SIGTERM không đủ)
        # + timer tương ứng — quản lý tách khỏi self._proc để stop() có thể
        # trả về ngay lập tức, không block executor.
        self._pending_kill_proc: Optional[subprocess.Popen] = None
        self._kill_timer = None

        self._resume_pub = node.create_publisher(Bool, '/explore/resume', 10)
        self._controller_selector_pub = node.create_publisher(
            String, '/controller_selector', _SELECTOR_QOS
        )

    # ── Public API ──────────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        """Tiến trình explore_lite còn sống hay không (kể cả đang pause)."""
        return self._proc is not None and self._proc.poll() is None

    @property
    def is_exploring(self) -> bool:
        return self.is_running and self._exploring

    def start(self, algo: str = 'dwa'):
        """Bắt đầu (hoặc resume nếu đang pause) frontier exploration."""
        self._current_algo = algo
        if not self.is_running:
            self._spawn_process()
        self._set_controller_id(algo)
        self._publish_resume(True)
        self._exploring = True
        self._slog.info(
            "explore_start", algo=algo,
            controller_id=ALGO_TO_CONTROLLER.get(algo, 'FollowPathDWA'),
        )

    def pause(self):
        """Tạm dừng tìm frontier, giữ tiến trình sống (lệnh 'pause')."""
        if self.is_running:
            self._publish_resume(False)
            self._exploring = False
            self._slog.info("explore_pause")

    def stop(self):
        """Dừng hẳn explore_lite (lệnh 'stop' hoặc chuyển sang patrol/goto).

        FIX: bản cũ gọi self._proc.wait(timeout=5.0) — BLOCKING call ngay
        trong callback /tb4/cmd của executor đơn luồng (rclpy.spin mặc
        định). Trong tối đa 5s đó node không xử lý được BẤT KỲ callback
        nào khác: heartbeat ngưng, service lifecycle get/set (dùng bởi CLI
        'status'/'activate'/'deactivate') bị treo, và lệnh /tb4/cmd tiếp
        theo (vd 'goto' gửi ngay sau khi tắt explore) bị xếp hàng chờ —
        đây là nguyên nhân "status không hoạt động", "activate/deactivate
        không hoạt động đúng sau explore". Ngoài ra 'ros2 launch' chỉ là
        tiến trình cha bọc explore_lite thật; SIGTERM cho riêng cha không
        đảm bảo con dừng kịp, nên explore_lite có thể kịp gửi thêm 1 goal
        frontier mới ngay khi bạn vừa gửi 'goto' — hai goal đá nhau.
        Bản mới: gửi SIGTERM cho CẢ process group ngay lập tức, không đợi,
        cập nhật is_running() = False tức thì, và chỉ SIGKILL cưỡng bức sau
        1.5s (qua timer không-block) nếu tiến trình vẫn còn sống.
        """
        if self._proc is None:
            self._exploring = False
            return
        self._slog.info("explore_stop")
        proc = self._proc
        self._proc = None
        self._exploring = False
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

        if self._kill_timer:
            self._kill_timer.cancel()
            self._node.destroy_timer(self._kill_timer)
            self._kill_timer = None
        self._pending_kill_proc = proc
        self._kill_timer = self._node.create_timer(1.5, self._force_kill_if_alive)

    def _force_kill_if_alive(self):
        if self._kill_timer:
            self._kill_timer.cancel()
            self._node.destroy_timer(self._kill_timer)
            self._kill_timer = None
        proc = self._pending_kill_proc
        self._pending_kill_proc = None
        if proc is not None and proc.poll() is None:
            self._slog.warn("explore_stop_force_kill")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    def set_algo(self, algo: str):
        """Đổi thuật toán lái khi đang explore, không cần restart tiến trình."""
        self._current_algo = algo
        if self.is_running:
            self._set_controller_id(algo)

    # ── Private ─────────────────────────────────────────────────────────────

    def _spawn_process(self):
        self._slog.info("explore_process_spawn")
        self._proc = subprocess.Popen(
            [
                'ros2', 'launch', 'explore_lite', 'explore.launch.py',
                'use_sim_time:=true',
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            # FIX: đặt tiến trình con vào process group riêng (setsid) để
            # stop() có thể os.killpg() diệt cả 'ros2 launch' LẪN tiến
            # trình explore_lite thật mà nó fork ra — SIGTERM cho riêng
            # PID cha (mặc định trước đây) không đảm bảo con bị dừng theo.
            preexec_fn=os.setsid,
        )

    def _publish_resume(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self._resume_pub.publish(msg)

    def _set_controller_id(self, algo: str):
        controller_id = ALGO_TO_CONTROLLER.get(algo, 'FollowPathDWA')

        # FIXED: QoS TRANSIENT_LOCAL (latched) thay cho vòng lặp time.sleep()
        # chờ subscriber — xem giải thích trong NavigationManager.send_goal().
        self._controller_selector_pub.publish(String(data=controller_id))
        self._slog.info("explore_controller_id_set", controller_id=controller_id)