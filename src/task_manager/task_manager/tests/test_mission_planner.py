"""
test_mission_planner.py — Unit tests for MissionPlanner.
"""

import pytest
import yaml
from task_manager.managers.mission_planner import MissionPlanner


class DummyLogger:
    def info(self, m): pass
    def error(self, m): pass
    def warn(self, m): pass
    def debug(self, m): pass


@pytest.fixture
def planner():
    return MissionPlanner(DummyLogger())


@pytest.fixture
def loop_yaml(tmp_path) -> str:
    cfg = {
        'patrol_sequence': ['A', 'B', 'C'],
        'loop_patrol': True,
        'waypoints': {
            'A': {'pose': {'x': 1.0, 'y': 0.0}, 'tasks': []},
            'B': {'pose': {'x': 2.0, 'y': 0.0}, 'tasks': []},
            'C': {'pose': {'x': 3.0, 'y': 0.0}, 'tasks': []},
        },
    }
    p = tmp_path / 'w.yaml'
    p.write_text(yaml.dump(cfg))
    return str(p)


@pytest.fixture
def noloop_yaml(tmp_path) -> str:
    cfg = {
        'patrol_sequence': ['A', 'B'],
        'loop_patrol': False,
        'waypoints': {
            'A': {'pose': {'x': 1.0, 'y': 0.0}, 'tasks': []},
            'B': {'pose': {'x': 2.0, 'y': 0.0}, 'tasks': []},
        },
        'emergency_rules': {'low_battery_action': 'dock'},
    }
    cfg['waypoints']['dock'] = {'pose': {'x': 0.0, 'y': 0.0}, 'tasks': []}
    p = tmp_path / 'noloop.yaml'
    p.write_text(yaml.dump(cfg))
    return str(p)


class TestLoad:
    def test_load_success(self, planner, loop_yaml):
        assert planner.load_config(loop_yaml) is True
        assert len(planner.waypoints) == 3

    def test_load_nonexistent(self, planner):
        assert planner.load_config('/nonexistent/file.yaml') is False

    def test_load_empty_file(self, planner, tmp_path):
        p = tmp_path / 'empty.yaml'
        p.write_text('')
        assert planner.load_config(str(p)) is False

    def test_mission_id_generated(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        assert len(planner.mission_id) > 0


class TestPatrolSequence:
    def test_initial_waypoint(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        assert planner.get_current_waypoint_name() == 'A'

    def test_advance_moves_forward(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        planner.advance_mission()
        assert planner.get_current_waypoint_name() == 'B'

    def test_loop_wraps_around(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        planner.advance_mission()
        planner.advance_mission()
        planner.advance_mission()
        assert planner.get_current_waypoint_name() == 'A'

    def test_no_loop_returns_none_at_end(self, planner, noloop_yaml):
        planner.load_config(noloop_yaml)
        planner.advance_mission()
        planner.advance_mission()
        assert planner.get_current_waypoint_name() is None

    def test_is_mission_complete_noloop(self, planner, noloop_yaml):
        planner.load_config(noloop_yaml)
        planner.advance_mission()
        planner.advance_mission()
        assert planner.is_mission_complete() is True

    def test_is_mission_complete_loop_never(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        planner.advance_mission()
        planner.advance_mission()
        planner.advance_mission()
        assert planner.is_mission_complete() is False


class TestDockWaypoint:
    def test_dock_waypoint_loaded(self, planner, noloop_yaml):
        planner.load_config(noloop_yaml)
        assert planner.dock_waypoint == 'dock'
        data = planner.get_dock_waypoint_data()
        assert data is not None

    def test_dock_waypoint_default_missing(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        # loop_yaml doesn't have emergency_rules, so default 'tram_sac'
        assert planner.dock_waypoint == 'tram_sac'

    def test_get_current_waypoint_data(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        data = planner.get_current_waypoint_data()
        assert data is not None
        assert 'pose' in data


class TestReset:
    def test_reset_returns_to_start(self, planner, loop_yaml):
        planner.load_config(loop_yaml)
        planner.advance_mission()
        old_id = planner.mission_id
        planner.reset()
        assert planner.current_idx == 0
        assert planner.mission_id != old_id
