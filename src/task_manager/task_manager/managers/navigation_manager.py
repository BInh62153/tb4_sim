#!/usr/bin/env python3
"""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
NavigationManager: gửi Nav2 goals, clear costmap, track goal handle.

Inject:
    node         – ROS2 LifecycleNode (để tạo ActionClient/ServiceClient)
    slog         – StructuredLogger
    state_machine – StateMachine (dispatch events)
"""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

#: QoS "latched": publisher giữ lại message cuối cùng, subscriber join trễ
#: (vd ControllerSelector BT node được nav2 tạo lại mỗi goal) vẫn nhận được
#: ngay lập tức — không cần vòng lặp chờ get_subscription_count() nữa.
_SELECTOR_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)

#: Map tên thuật toán (nhận từ tb4_cli / send_waypoints.py) sang controller_id
#: khai báo trong config/nav2/nav2_params.yaml (controller_plugins).
ALGO_TO_CONTROLLER = {
    'dwa':     'FollowPathDWA',
    'teb':     'FollowPathTEB',
    'pp':      'FollowPathPP',
    'stanley': 'FollowPathStanley',
}


class NavigationManager:
    """Wrapper quanh Nav2 NavigateToPose action client."""

    def __init__(self, node, slog):
        self._node = node
        self._slog = slog

        self._nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self._clear_local_srv = node.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )
        self._clear_global_srv = node.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self._current_goal_handle = None
        self._nav_start_time: Optional[float] = None
        self._controller_selector_pub = node.create_publisher(
            String, '/controller_selector', _SELECTOR_QOS
        )
        # FIX: đánh dấu khi operator chủ động cancel (stop/pause/preempt) để
        # phân biệt với thất bại thật sự. Nếu không có cờ này, mọi cancel()
        # đều khiến _on_result() trả success=False -> dispatch NAV_FAILED ->
        # trigger toàn bộ recovery pipeline (spin 90°, backup 0.2m...) NGAY
        # CẢ KHI người dùng chỉ muốn dừng — đây là lý do 'stop' "không dừng
        # được" trong khi goto: robot vẫn tiếp tục quay/lùi vài giây sau.
        self._cancel_requested = False

    # ── Server readiness ───────────────────────────────────────────────────

    def wait_for_server(self, timeout: float = 5.0) -> bool:
        return self._nav_client.wait_for_server(timeout_sec=timeout)

    # ── Goal management ────────────────────────────────────────────────────

    def send_goal(
        self,
        wp_data: Dict,
        on_done: Callable[[bool], None],
        goal_id: str = "",
        algo: str = "dwa",
    ) -> bool:
        """
        Gửi NavigateToPose goal.
        on_done(success: bool) được gọi khi goal kết thúc.

        algo: 'dwa' | 'teb' | 'pp' | 'stanley' — chọn controller plugin
              (map sang controller_id qua ALGO_TO_CONTROLLER). Không hợp lệ
              hoặc rỗng sẽ fallback về FollowPathDWA.
        """
        if not self._nav_client.server_is_ready():
            self._slog.warn("nav_server_not_ready", goal_id=goal_id)
            return False

        pose  = wp_data.get('pose', {})
        goal  = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self._node.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(pose.get('x', 0.0))
        goal.pose.pose.position.y = float(pose.get('y', 0.0))
        goal.pose.pose.position.z = float(pose.get('z', 0.0))

        yaw = float(pose.get('yaw', 0.0))
        cy  = math.cos(yaw * 0.5)
        sy  = math.sin(yaw * 0.5)
        goal.pose.pose.orientation = Quaternion(w=cy, x=0.0, y=0.0, z=sy)

        controller_id = ALGO_TO_CONTROLLER.get((algo or "").lower(), 'FollowPathDWA')

        # FIXED: publisher dùng QoS TRANSIENT_LOCAL (latched) nên không cần
        # chờ get_subscription_count() nữa — subscriber (ControllerSelector BT
        # node, được nav2 tạo lại mỗi goal) join trễ vẫn nhận được message cuối
        # ngay lập tức. Trước đây vòng lặp time.sleep() ở đây block TOÀN BỘ
        # single-threaded executor của node (không nhận lệnh CLI, không xử lý
        # callback nav2/heartbeat) tới 0.5s MỖI LẦN gửi goal — chính là nguyên
        # nhân patrol/goto "phản ứng trễ" và CLI "gõ lệnh phải chờ lâu".
        self._controller_selector_pub.publish(String(data=controller_id))

        self._nav_start_time = time.monotonic()
        self._slog.info("nav_goal_sent", goal_id=goal_id,
                        x=pose.get('x'), y=pose.get('y'),
                        algo=algo, controller_id=controller_id)

        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(
            lambda f: self._on_goal_response(f, on_done, goal_id)
        )
        return True

    def cancel(self):
        if self._current_goal_handle is not None:
            self._cancel_requested = True
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self._slog.info("nav_goal_cancelled")

    # ── Costmap clearing ───────────────────────────────────────────────────

    def clear_local_costmap(self, callback: Callable):
        req = ClearEntireCostmap.Request()
        self._clear_local_srv.call_async(req).add_done_callback(lambda _: callback())

    def clear_global_costmap(self, callback: Callable):
        req = ClearEntireCostmap.Request()
        self._clear_global_srv.call_async(req).add_done_callback(lambda _: callback())

    def clear_both_costmaps(self, callback: Callable):
        """Xóa cả hai costmap song song, gọi callback sau khi cả hai xong."""
        req    = ClearEntireCostmap.Request()
        status = {'done': 0}

        def _check(_):
            status['done'] += 1
            if status['done'] == 2:
                callback()

        self._clear_local_srv.call_async(req).add_done_callback(_check)
        self._clear_global_srv.call_async(req).add_done_callback(_check)

    # ── Private callbacks ──────────────────────────────────────────────────

    def _on_goal_response(self, future, on_done, goal_id):
        handle = future.result()
        if not handle.accepted:
            self._slog.warn("nav_goal_rejected", goal_id=goal_id)
            on_done(False)
            return
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, on_done, goal_id)
        )

    def _on_result(self, future, on_done, goal_id):
        self._current_goal_handle = None
        elapsed = (
            round(time.monotonic() - self._nav_start_time, 2)
            if self._nav_start_time else -1
        )
        result = future.result()

        # FIX: nếu goal bị CANCELED do chính operator (stop/pause/goto mới
        # đè lên) thì KHÔNG dispatch on_done() -> không trigger NAV_FAILED
        # -> không chạy recovery (spin/backup). Trước đây mọi cancel đều
        # rơi vào nhánh success=False như một lỗi thật, khiến robot tự
        # quay/lùi ngay sau khi người dùng bấm 'stop'.
        if result.status == GoalStatus.STATUS_CANCELED and self._cancel_requested:
            self._cancel_requested = False
            self._slog.info("nav_goal_canceled_by_operator", goal_id=goal_id)
            return

        self._cancel_requested = False
        success = result.status == GoalStatus.STATUS_SUCCEEDED
        self._slog.info(
            "nav_goal_result",
            goal_id=goal_id,
            success=success,
            nav_time_s=elapsed,
        )
        on_done(success)