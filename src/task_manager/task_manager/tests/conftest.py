"""
conftest.py — shared fixtures for task_manager tests.
"""

import sys
from types import ModuleType
from unittest.mock import MagicMock

import pytest
import yaml


def _install_ros_stubs() -> None:
    """Allow importing task_manager without a sourced ROS2 environment."""
    if 'rclpy' in sys.modules:
        return

    def _mod(name: str) -> ModuleType:
        parts = name.split('.')
        for i in range(1, len(parts)):
            _mod('.'.join(parts[:i]))
        if name not in sys.modules:
            mod = ModuleType(name)
            sys.modules[name] = mod
            parent, _, child = name.rpartition('.')
            if parent and child:
                setattr(sys.modules[parent], child, mod)
        return sys.modules[name]

    rclpy = _mod('rclpy')
    rclpy.ok = lambda: True
    rclpy_action = _mod('rclpy.action')
    rclpy_action.ActionClient = MagicMock

    sensor_msg = _mod('sensor_msgs.msg')

    class BatteryState:
        POWER_SUPPLY_STATUS_CHARGING = 1
        POWER_SUPPLY_STATUS_DISCHARGING = 2
        POWER_SUPPLY_STATUS_FULL = 4

    sensor_msg.BatteryState = BatteryState

    action_msg = _mod('action_msgs.msg')

    class GoalStatus:
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

    action_msg.GoalStatus = GoalStatus

    geometry_msg = _mod('geometry_msgs.msg')

    class _Header:
        def __init__(self):
            self.frame_id = ''
            self.stamp = None

    class _Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = None

    class PoseStamped:
        def __init__(self):
            self.header = _Header()
            self.pose = _Pose()

    geometry_msg.PoseStamped = PoseStamped
    geometry_msg.Quaternion = lambda **kw: type('Quaternion', (), kw)()

    nav2_action = _mod('nav2_msgs.action')

    def _action_type():
        class ActionType:
            class Goal:
                def __init__(self):
                    self.pose = PoseStamped()

            class Result:
                pass

            class Feedback:
                pass
        return ActionType

    nav2_action.NavigateToPose = _action_type()
    nav2_action.Spin = _action_type()
    nav2_action.BackUp = _action_type()

    nav2_srv = _mod('nav2_msgs.srv')

    class ClearEntireCostmap:
        class Request:
            pass

        class Response:
            pass

    nav2_srv.ClearEntireCostmap = ClearEntireCostmap

    # ── Stubs cho ExploreManager (std_msgs.msg.Bool + rcl_interfaces) ───────
    std_msg = _mod('std_msgs.msg')

    class Bool:
        def __init__(self):
            self.data = False

    std_msg.Bool = Bool

    rcl_iface_msg = _mod('rcl_interfaces.msg')

    class ParameterType:
        PARAMETER_NOT_SET = 0
        PARAMETER_BOOL = 1
        PARAMETER_INTEGER = 2
        PARAMETER_DOUBLE = 3
        PARAMETER_STRING = 4

    class ParameterValue:
        def __init__(self, **kw):
            for k, v in kw.items():
                setattr(self, k, v)

    class Parameter:
        def __init__(self):
            self.name = ''
            self.value = None

    rcl_iface_msg.ParameterType = ParameterType
    rcl_iface_msg.ParameterValue = ParameterValue
    rcl_iface_msg.Parameter = Parameter

    rcl_iface_srv = _mod('rcl_interfaces.srv')

    class SetParameters:
        class Request:
            def __init__(self):
                self.parameters = []

        class Response:
            def __init__(self):
                self.results = []

    rcl_iface_srv.SetParameters = SetParameters


_install_ros_stubs()


class DummyLogger:
    """Minimal logger for unit tests."""
    def info(self, m):  pass
    def warn(self, m):  pass
    def error(self, m): pass
    def debug(self, m): pass


@pytest.fixture
def logger():
    return DummyLogger()


@pytest.fixture
def minimal_yaml(tmp_path) -> str:
    cfg = {
        'patrol_sequence': ['A', 'B'],
        'loop_patrol': False,
        'waypoints': {
            'A': {'pose': {'x': 1.0, 'y': 0.0, 'yaw': 0.0}, 'tasks': []},
            'B': {'pose': {'x': 2.0, 'y': 0.0, 'yaw': 0.0}, 'tasks': []},
        },
        'emergency_rules': {'low_battery_action': 'dock'},
    }
    cfg['waypoints']['dock'] = {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'tasks': []}
    p = tmp_path / 'waypoints.yaml'
    p.write_text(yaml.dump(cfg))
    return str(p)