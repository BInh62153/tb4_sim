#!/usr/bin/env python3
"""
TurtleBot4LifecycleNode — thin orchestrator.

Trách nhiệm:
  • Lifecycle transitions (on_configure / on_activate / on_deactivate / …)
  • Khởi tạo + inject dependencies vào tất cả managers
  • Đăng ký Event handlers trên StateMachine
  • Subscribe /tb4/cmd → parse → dispatch events
  • Publish heartbeat

Không có business logic ở đây — tất cả nằm trong managers/.

"""

import uuid

import rclpy
import rclpy.parameter
from rclpy.action import ActionClient
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State as LCState

from nav2_msgs.action import Spin
from std_msgs.msg import String

from .state.states import SystemState
from .state.state_machine import StateMachine, Event
from .state.structured_logger import StructuredLogger
from .managers.mission_planner   import MissionPlanner
from .managers.navigation_manager import NavigationManager
from .managers.recovery_manager   import RecoveryManager
from .managers.battery_manager    import BatteryManager
from .managers.dock_manager       import DockManager
from .managers.task_executor      import TaskExecutor
from .managers.explore_manager    import ExploreManager


class TurtleBot4LifecycleNode(LifecycleNode):
    """
    Thin lifecycle orchestrator.
    Wires: StateMachine ↔ EventBus ↔ Managers.
    """

    def __init__(self):
        super().__init__('task_manager_lifecycle_node')

        # Parameters (declare once in __init__) 
        self.declare_parameter('waypoints_file', '/ros2_ws/config/waypoints.yaml')
        self.declare_parameter('low_battery_pct',     20.0)
        self.declare_parameter('resume_battery_pct',  80.0)
        self.declare_parameter('critical_battery_pct', 10.0)

        # Structured logger
        self._slog = StructuredLogger(self.get_logger(), mission_id="")

        # State machine
        self._sm = StateMachine(self.get_logger(), structured_log=self._slog.info)

        # Shared resources
        self._spin_client = ActionClient(self, Spin, 'spin')

        # Managers (inject dependencies) 
        self._planner   = MissionPlanner(self.get_logger(), self.get_clock())
        self._nav_mgr   = NavigationManager(self, self._slog)
        self._recovery  = RecoveryManager(self, self._nav_mgr, self._spin_client, self._slog)
        self._battery   = BatteryManager(self, self._slog)
        self._dock_mgr  = DockManager(self, self._nav_mgr, self._battery, self._slog)
        self._executor  = TaskExecutor(self, self._spin_client, self._slog)
        self._explore_mgr = ExploreManager(self, self._slog)

        # ── ROS interfaces ──────────────────────────────────────────────────
        self._status_pub  = self.create_lifecycle_publisher(String, '/tb4/status', 10)
        self._heartbeat   = None
        self._cmd_sub     = None

        # Thuật toán controller hiện hành (dwa/teb/pp/stanley), do CLI chọn
        # qua lệnh 'patrol:<algo>' hoặc 'goto:<wp>:<algo>'. Dùng cho mọi goal
        # gửi trong _cycle() (mission tự động) lẫn goto thủ công.
        self._current_algo = 'dwa'

        # 'mission' | 'explore' | None — nhớ lệnh 'pause' vừa dừng luồng nào,
        # để 'resume' biết nên tiếp tục mission cycle hay explore.
        self._paused_from = None

        # ── Register event handlers ────────────────────────────────────────
        self._register_events()

    # ═══════════════════════════════════════════════════════════════════════
    #  Event registration
    # ═══════════════════════════════════════════════════════════════════════

    def _register_events(self):
        sm = self._sm
        sm.register(Event.NAV_SUCCESS,        self._on_nav_success)
        sm.register(Event.NAV_FAILED,         self._on_nav_failed)
        sm.register(Event.TASKS_COMPLETE,     self._on_tasks_complete)
        sm.register(Event.RECOVERY_SUCCESS,   self._on_recovery_success)
        sm.register(Event.RECOVERY_ABORTED,   self._on_recovery_aborted)
        sm.register(Event.BATTERY_LOW,        self._on_battery_low)
        sm.register(Event.BATTERY_CRITICAL,   self._on_battery_critical)
        sm.register(Event.CHARGE_COMPLETE,    self._on_charge_complete)
        sm.register(Event.CMD_PATROL,         self._on_cmd_patrol)
        sm.register(Event.CMD_PAUSE,          self._on_cmd_pause)
        sm.register(Event.CMD_STOP,           self._on_cmd_stop)
        sm.register(Event.CMD_GOTO,           self._on_cmd_goto)
        sm.register(Event.CMD_RESUME,         self._on_cmd_resume)
        sm.register(Event.CMD_EXPLORE,        self._on_cmd_explore)

    # ═══════════════════════════════════════════════════════════════════════
    #  Lifecycle callbacks ///////////////////
    # ═══════════════════════════════════════════════════════════════════════

    def on_configure(self, state: LCState) -> TransitionCallbackReturn:
        self.get_logger().info("[LC] Configuring...")

        wp_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        if not self._planner.load_config(wp_file):
            return TransitionCallbackReturn.FAILURE

        self._slog.mission_id = self._planner.mission_id

        if not self._nav_mgr.wait_for_server(timeout=2.0):
            self.get_logger().warn("[LC] Nav2 server not ready during configure — continuing.")

        self._sm.force_transition(SystemState.IDLE, "on_configure")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LCState) -> TransitionCallbackReturn:
        self.get_logger().info("[LC] Activating...")

        params = {
            'low_battery_pct':      self.get_parameter('low_battery_pct').value,
            'resume_battery_pct':   self.get_parameter('resume_battery_pct').value,
            'critical_battery_pct': self.get_parameter('critical_battery_pct').value,
        }
        self._battery.reconfigure(params)
        self._battery.start(
            on_low      = lambda: self._sm.dispatch(Event.BATTERY_LOW),
            on_critical  = lambda: self._sm.dispatch(Event.BATTERY_CRITICAL),
            on_charged   = lambda: self._sm.dispatch(Event.CHARGE_COMPLETE),
        )

        self._cmd_sub   = self.create_subscription(
            String, '/tb4/cmd', self._on_cmd_received, 10
        )
        self._heartbeat = self.create_timer(1.0, self._publish_heartbeat)

        self._sm.force_transition(SystemState.IDLE, "on_activate")
        self._cycle()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LCState) -> TransitionCallbackReturn:
        self.get_logger().info("[LC] Deactivating...")
        self._battery.stop()
        self._nav_mgr.cancel()
        self._executor.cancel()
        self._dock_mgr.cancel()
        self._explore_mgr.stop()
        if self._cmd_sub:
            self.destroy_subscription(self._cmd_sub)
            self._cmd_sub = None
        if self._heartbeat:
            self._heartbeat.cancel()
            self.destroy_timer(self._heartbeat)
            self._heartbeat = None
        self._sm.force_transition(SystemState.PAUSED, "on_deactivate")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LCState) -> TransitionCallbackReturn:
        self.get_logger().info("[LC] Cleaning up...")
        self._sm.force_transition(SystemState.UNCONFIGURED, "on_cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LCState) -> TransitionCallbackReturn:
        self.get_logger().error("[LC] Error handler triggered.")
        self._nav_mgr.cancel()
        self._battery.stop()
        self._sm.force_transition(SystemState.ABORTED, "on_error")
        return TransitionCallbackReturn.SUCCESS

    # ═══════════════════════════════════════════════════════════════════════
    #  Core mission cycle
    # ═══════════════════════════════════════════════════════════════════════

    def _cycle(self):
        """Fire-and-forget: chạy bước tiếp theo của mission nếu đang IDLE."""
        if not self._sm.is_in(SystemState.IDLE):
            return

        if self._battery.is_low or self._battery.is_critical:
            self._sm.transition(SystemState.LOW_BATTERY_DOCKING, "battery_low")
            self._do_dock()
            return

        wp_name = self._planner.get_current_waypoint_name()
        if wp_name is None:
            self.get_logger().info("[Mission] All waypoints done.")
            return

        wp_data = self._planner.get_current_waypoint_data()
        if wp_data is None:
            self.get_logger().warn(f"[Mission] Missing data for '{wp_name}', skipping.")
            self._planner.advance_mission()
            self._cycle()
            return

        goal_id = str(uuid.uuid4())[:8]
        self._slog.goal_id  = goal_id
        self._slog.waypoint = wp_name
        self._sm.transition(SystemState.NAVIGATING, f"wp={wp_name}")

        ok = self._nav_mgr.send_goal(
            wp_data,
            on_done = lambda success: self._sm.dispatch(
                Event.NAV_SUCCESS if success else Event.NAV_FAILED,
                wp_data=wp_data,
                wp_name=wp_name,
            ),
            goal_id=goal_id,
            algo=self._current_algo,
        )
        if not ok:
            self._sm.transition(SystemState.RECOVERING, "nav_send_failed")
            self._do_recovery()

    # ═══════════════════════════════════════════════════════════════════════
    #  Event handlers
    # ═══════════════════════════════════════════════════════════════════════

    def _on_nav_success(self, wp_data, wp_name, **_):
        self._recovery.reset()
        self._sm.transition(SystemState.EXECUTING_TASK, f"arrived={wp_name}")
        self._executor.execute(
            tasks       = wp_data.get('tasks', []),
            on_complete = lambda: self._sm.dispatch(Event.TASKS_COMPLETE),
            waypoint    = wp_name,
        )

    def _on_nav_failed(self, **_):
        self._sm.transition(SystemState.RECOVERING, "nav_failed")
        self._do_recovery()

    def _on_tasks_complete(self, **_):
        self._planner.advance_mission()
        self._sm.transition(SystemState.IDLE, "tasks_done")
        self._cycle()

    def _on_recovery_success(self, **_):
        self._sm.transition(SystemState.IDLE, "recovery_ok")
        self._cycle()

    def _on_recovery_aborted(self, **_):
        self._planner.advance_mission()
        self._sm.transition(SystemState.IDLE, "recovery_abort_skip_wp")
        self._cycle()

    def _on_battery_low(self, **_):
        if self._sm.is_in(SystemState.NAVIGATING, SystemState.EXECUTING_TASK, SystemState.EXPLORING):
            self._nav_mgr.cancel()
            self._executor.cancel()
            self._explore_mgr.stop()
            self._sm.transition(SystemState.LOW_BATTERY_DOCKING, "battery_low_interrupt")
            self._do_dock()

    def _on_battery_critical(self, **_):
        self.get_logger().error("[Battery] CRITICAL — hard interrupt!")
        self._nav_mgr.cancel()
        self._executor.cancel()
        self._explore_mgr.stop()
        self._sm.force_transition(SystemState.LOW_BATTERY_DOCKING, "battery_critical")
        self._do_dock()

    def _on_charge_complete(self, **_):
        self._sm.transition(SystemState.RESUME_AFTER_CHARGE, "charge_done")
        self._sm.transition(SystemState.IDLE, "resume_mission")
        self._cycle()

    # ── Operator command events ─────────────────────────────────────────────

    def _on_cmd_patrol(self, algo: str = "dwa", **_):
        if self._sm.is_in(SystemState.IDLE, SystemState.PAUSED, SystemState.EXPLORING):
            self._explore_mgr.stop()
            self._current_algo = algo
            self._sm.force_transition(SystemState.IDLE, "cmd_patrol")
            self._cycle()

    def _on_cmd_pause(self, **_):
        if self._sm.is_in(SystemState.EXPLORING):
            self._explore_mgr.pause()
            self._paused_from = 'explore'
        else:
            self._paused_from = 'mission'
        self._nav_mgr.cancel()
        self._sm.force_transition(SystemState.PAUSED, "cmd_pause")

    def _on_cmd_stop(self, **_):
        self._nav_mgr.cancel()
        self._executor.cancel()
        self._explore_mgr.stop()
        self._paused_from = None
        self._sm.force_transition(SystemState.PAUSED, "cmd_stop")

    def _on_cmd_goto(self, wp_name: str = "", algo: str = "dwa", **_):
        wp_data = self._planner.waypoints.get(wp_name)
        if not wp_data:
            self.get_logger().error(f"[CMD] goto: waypoint '{wp_name}' not found.")
            return
        self._explore_mgr.stop()
        self._current_algo = algo
        self._nav_mgr.cancel()
        goal_id = str(uuid.uuid4())[:8]
        self._slog.goal_id  = goal_id
        self._slog.waypoint = wp_name
        self._sm.force_transition(SystemState.NAVIGATING, f"cmd_goto={wp_name}")
        self._nav_mgr.send_goal(
            wp_data,
            on_done = lambda success: self._sm.dispatch(
                Event.NAV_SUCCESS if success else Event.NAV_FAILED,
                wp_data=wp_data,
                wp_name=wp_name,
            ),
            goal_id=goal_id,
            algo=algo,
        )

    def _on_cmd_resume(self, **_):
        if not self._sm.is_in(SystemState.PAUSED):
            return
        if self._paused_from == 'explore':
            self._sm.force_transition(SystemState.EXPLORING, "cmd_resume_explore")
            self._explore_mgr.start(self._current_algo)
        else:
            self._sm.force_transition(SystemState.IDLE, "cmd_resume")
            self._cycle()
        self._paused_from = None

    def _on_cmd_explore(self, algo: str = "dwa", **_):
        if self._sm.is_in(SystemState.IDLE, SystemState.PAUSED):
            self._current_algo = algo
            self._sm.force_transition(SystemState.EXPLORING, "cmd_explore")
            self._explore_mgr.start(algo)
        elif self._sm.is_in(SystemState.EXPLORING):
            # Đang explore rồi -> chỉ đổi thuật toán, không restart tiến trình.
            self._current_algo = algo
            self._explore_mgr.set_algo(algo)
        else:
            self.get_logger().warn(
                f"[CMD] explore: bỏ qua vì đang ở state {self._sm.state.name}"
            )

    # ═══════════════════════════════════════════════════════════════════════
    #  Operator command subscription
    # ═══════════════════════════════════════════════════════════════════════

    def _on_cmd_received(self, msg: String):
        text = msg.data.strip()
        self.get_logger().info(f"[CMD] Received: '{text}'")

        # tb4_cli.py gửi lệnh dạng "base[:arg1[:arg2]]", ví dụ:
        #   "patrol:dwa"        -> base=patrol, algo=dwa
        #   "goto:diem_A:teb"   -> base=goto,   wp_name=diem_A, algo=teb
        #   "explore:dwa"       -> base=explore (chưa có backend, xem dưới)
        #   "stop" / "pause" / "resume" -> không có hậu tố
        parts = [p.strip() for p in text.split(':')]
        base  = parts[0].lower()

        if base == 'patrol':
            algo = parts[1].lower() if len(parts) > 1 and parts[1] else 'dwa'
            self._sm.dispatch(Event.CMD_PATROL, algo=algo)

        elif base == 'pause':
            self._sm.dispatch(Event.CMD_PAUSE)

        elif base == 'stop':
            self._sm.dispatch(Event.CMD_STOP)

        elif base == 'resume':
            self._sm.dispatch(Event.CMD_RESUME)

        elif base == 'goto':
            if len(parts) < 2 or not parts[1]:
                self.get_logger().warn(f"[CMD] goto thiếu tên waypoint: '{text}'")
                return
            wp_name = parts[1]
            algo = parts[2].lower() if len(parts) > 2 and parts[2] else 'dwa'
            self._sm.dispatch(Event.CMD_GOTO, wp_name=wp_name, algo=algo)

        elif base == 'explore':
            algo = parts[1].lower() if len(parts) > 1 and parts[1] else 'dwa'
            self._sm.dispatch(Event.CMD_EXPLORE, algo=algo)

        else:
            self.get_logger().warn(f"[CMD] Unknown command: '{text}'")

    # ═══════════════════════════════════════════════════════════════════════
    #  Internal helpers
    # ═══════════════════════════════════════════════════════════════════════

    def _do_dock(self):
        dock_data = self._planner.get_dock_waypoint_data()
        self._sm.transition(SystemState.CHARGING, "docking")
        self._dock_mgr.start_docking(
            dock_wp_data = dock_data,
            on_complete  = self._on_dock_finished,
        )

    def _on_dock_finished(self, success: bool):
        if success:
            self._sm.dispatch(Event.CHARGE_COMPLETE)
        else:
            self.get_logger().error("[Dock] Docking failed.")
            self._sm.force_transition(SystemState.IDLE, "dock_failed_resume")
            self._cycle()

    def _do_recovery(self):
        self._recovery.start(on_complete=self._on_recovery_done)

    def _on_recovery_done(self, success: bool):
        event = Event.RECOVERY_SUCCESS if success else Event.RECOVERY_ABORTED
        self._sm.dispatch(event)

    # ═══════════════════════════════════════════════════════════════════════
    #  Heartbeat
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_heartbeat(self):
        if self._status_pub.is_activated:
            msg      = String()
            msg.data = (
                f"state={self._sm.state.name} | "
                f"{self._planner.status_summary()} | "
                f"bat={self._battery.status_str()}"
            )
            self._status_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4LifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Dừng Task Manager Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()