#!/usr/bin/env python3
"""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ExploreManager: khởi động/dừng node `explore_lite` (package m-explore-ros2)
và đồng bộ lựa chọn thuật toán (algo) với controller_id của nó — cùng cách
NavigationManager làm cho goto/patrol.

explore_lite là 1 node C++ độc lập (không chạy trong tiến trình Python của
task_manager), nên được quản lý theo 3 cơ chế:

  • start/stop tiến trình  → spawn/kill qua `ros2 launch explore_lite
    explore.launch.py` (subprocess).
  • pause/resume tìm frontier (không kill tiến trình) → publish
    std_msgs/Bool lên topic `/explore/resume` (đã có sẵn trong explore.cpp,
    xem Explore::resumeCallback — không cần sửa).
  • chọn thuật toán lái (dwa/teb/pp/stanley) → set parameter `controller_id`
    trên node `explore_node` qua service `/explore_node/set_parameters`.
    (yêu cầu patch nhỏ trong src/m-explore-ros2/explore/src/explore.cpp +
    include/explore/explore.h để node đọc parameter này khi gửi goal —
    xem PATCH đi kèm. Nếu chưa patch, explore vẫn chạy được, chỉ luôn dùng
    FollowPathDWA mặc định.)

Inject:
    node  – ROS2 LifecycleNode (để tạo publisher/service client)
    slog  – StructuredLogger
"""

from __future__ import annotations

import subprocess
from typing import Optional

from std_msgs.msg import Bool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


#: Map tên thuật toán sang controller_id (giống ALGO_TO_CONTROLLER trong
#: navigation_manager.py — giữ 1 nguồn sự thật sẽ tốt hơn nếu sau này gộp
#: 2 module lại, tạm thời lặp lại cho ExploreManager độc lập với Nav2 action).
ALGO_TO_CONTROLLER = {
    'dwa':     'FollowPathDWA',
    'teb':     'FollowPathTEB',
    'pp':      'FollowPathPP',
    # 'stanley': 'FollowPathStanley',  # chưa implement — xem TODO trong nav2_params.yaml
}

EXPLORE_NODE_NAME = 'explore_node'  # đặt trong explore.launch.py (name="explore_node")


class ExploreManager:
    """Điều phối vòng đời + thuật toán của node explore_lite."""

    SET_PARAM_TIMEOUT_S = 3.0

    def __init__(self, node, slog):
        self._node = node
        self._slog = slog

        self._proc: Optional[subprocess.Popen] = None
        self._exploring = False   # đang active tìm frontier (chưa pause)
        self._current_algo = 'dwa'

        self._resume_pub = node.create_publisher(Bool, '/explore/resume', 10)
        self._set_param_client = node.create_client(
            SetParameters, f'/{EXPLORE_NODE_NAME}/set_parameters'
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
        """Dừng hẳn explore_lite (lệnh 'stop' hoặc chuyển sang patrol/goto)."""
        if self._proc is None:
            self._exploring = False
            return
        self._slog.info("explore_stop")
        try:
            self._proc.terminate()
            self._proc.wait(timeout=5.0)
        except Exception as exc:
            self._slog.warn("explore_stop_force_kill", error=str(exc))
            self._proc.kill()
        self._proc = None
        self._exploring = False

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
        )

    def _publish_resume(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self._resume_pub.publish(msg)

    def _set_controller_id(self, algo: str):
        controller_id = ALGO_TO_CONTROLLER.get(algo, 'FollowPathDWA')

        if not self._set_param_client.wait_for_service(timeout_sec=self.SET_PARAM_TIMEOUT_S):
            self._slog.warn(
                "explore_set_param_unavailable",
                note="explore_node chưa sẵn sàng — sẽ dùng controller_id mặc định (FollowPathDWA) "
                     "cho tới lần đổi thuật toán kế tiếp",
            )
            return

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'controller_id'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=controller_id,
        )
        req.parameters = [param]

        fut = self._set_param_client.call_async(req)
        fut.add_done_callback(
            lambda f: self._on_set_param_done(f, controller_id)
        )

    def _on_set_param_done(self, future, controller_id: str):
        try:
            result = future.result()
            ok = bool(result.results) and result.results[0].successful
        except Exception as exc:
            self._slog.warn("explore_set_param_failed", error=str(exc))
            return
        if ok:
            self._slog.info("explore_controller_id_set", controller_id=controller_id)
        else:
            reason = result.results[0].reason if result.results else "unknown"
            self._slog.warn("explore_set_param_rejected", controller_id=controller_id, reason=reason)