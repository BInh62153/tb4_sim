"""
conftest.py — shared fixtures for task_manager tests.
"""

import pytest
import yaml


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
