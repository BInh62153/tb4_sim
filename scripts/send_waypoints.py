#!/usr/bin/env python3
"""
send_waypoints.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Script tiện ích: gửi một danh sách waypoint
thẳng đến Nav2 NavigateThroughPoses action.

Không cần task_manager, dùng được ngay sau khi
Nav2 đã khởi động.

Usage:
  python3 send_waypoints.py --waypoints diem_A diem_B diem_C
  python3 send_waypoints.py --patrol    # Dùng patrol_sequence trong YAML
  python3 send_waypoints.py --goto diem_B
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion

import yaml
import math
import sys
import argparse
from pathlib import Path


WAYPOINTS_FILE = '/ros2_ws/config/waypoints.yaml'


def yaw_to_quat(yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return Quaternion(w=cy, x=0.0, y=0.0, z=sy)


def load_waypoints(path: str) -> tuple[dict, list]:
    with open(path) as f:
        data = yaml.safe_load(f)
    wps = {}
    for name, d in data.get('waypoints', {}).items():
        pose = d['pose']
        wps[name] = {
            'x': float(pose['x']),
            'y': float(pose['y']),
            'yaw': float(pose.get('yaw', 0.0)),
            'label': d.get('label', name),
        }
    seq = data.get('patrol_sequence', list(wps.keys()))
    return wps, seq


def make_pose(wp: dict, clock) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = clock.now().to_msg()
    msg.pose.position.x = wp['x']
    msg.pose.position.y = wp['y']
    msg.pose.position.z = 0.0
    msg.pose.orientation = yaw_to_quat(wp['yaw'])
    return msg


class WaypointSender(Node):
    def __init__(self, mode: str, targets: list):
        super().__init__('waypoint_sender')
        self.mode = mode
        self.targets = targets

        self._through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._to_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.waypoints, self.patrol_seq = load_waypoints(WAYPOINTS_FILE)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

    def run(self):
        if self.mode == 'goto':
            wp_name = self.targets[0]
            if wp_name not in self.waypoints:
                self.get_logger().error(f'Waypoint "{wp_name}" không tồn tại!')
                return
            self._send_single(wp_name)
        elif self.mode in ('through', 'patrol'):
            seq = self.patrol_seq if self.mode == 'patrol' else self.targets
            self._send_through(seq)

    def _send_single(self, wp_name: str):
        self.get_logger().info(f'Gửi goal: {wp_name}')
        if not self._to_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('NavigateToPose server không khả dụng!')
            return

        wp = self.waypoints[wp_name]
        goal = NavigateToPose.Goal()
        goal.pose = make_pose(wp, self.get_clock())

        future = self._to_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal bị từ chối!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Đã đến đích!')

    def _send_through(self, seq: list):
        valid = [n for n in seq if n in self.waypoints]
        if not valid:
            self.get_logger().error('Không có waypoint hợp lệ!')
            return

        self.get_logger().info(f'Gửi {len(valid)} waypoints: {valid}')
        if not self._through_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('NavigateThroughPoses server không khả dụng!')
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = [make_pose(self.waypoints[n], self.get_clock()) for n in valid]

        future = self._through_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal bị từ chối!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Hoàn thành tất cả waypoints!')


def main():
    parser = argparse.ArgumentParser(description='TurtleBot4 Waypoint Sender')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--goto', metavar='WP', help='Đi đến 1 waypoint')
    group.add_argument('--waypoints', nargs='+', metavar='WP', help='Đi qua nhiều waypoints')
    group.add_argument('--patrol', action='store_true', help='Dùng patrol_sequence từ YAML')

    args = parser.parse_args()

    rclpy.init()

    if args.goto:
        node = WaypointSender('goto', [args.goto])
    elif args.waypoints:
        node = WaypointSender('through', args.waypoints)
    else:
        node = WaypointSender('patrol', [])

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
