#!/usr/bin/env python3
"""
ROS 2 mock publishers for Mission Timer bridge debugging.

- Publishes std_msgs/Bool to returned topics for robots (toggles every 5s)
- Publishes std_msgs/String to QR topic every 7s

Usage:
  source /opt/ros/<distro>/setup.bash
  python tools/ros2_mock_publishers.py \
    --config apps/mission-timer/backend/config.example.yaml

If --config is omitted, defaults are used:
  robots: Robot A (/robot_a/returned), Robot B (/robot_b/returned)
  qr_topic: /mission/qr
"""
import argparse
import time
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MockNode(Node):
    def __init__(self, config: dict):
        super().__init__('mission_timer_mock_publishers')
        self.robots = config.get('robots') or [
            {'name': 'Robot A', 'returned_topic': '/robot_a/returned'},
            {'name': 'Robot B', 'returned_topic': '/robot_b/returned'},
        ]
        self.qr_topic = config.get('qr_topic', '/mission/qr')

        self.robot_pubs = []
        self.robot_states = {}
        for r in self.robots:
            pub = self.create_publisher(Bool, r['returned_topic'], 10)
            self.robot_pubs.append((r['name'], r['returned_topic'], pub))
            self.robot_states[r['name']] = False
            self.get_logger().info(f"Publishing Bool on {r['returned_topic']} for {r['name']}")

        self.qr_pub = self.create_publisher(String, self.qr_topic, 10)
        self.get_logger().info(f"Publishing String on {self.qr_topic}")

        self.i = 0
        self.timer_robot = self.create_timer(5.0, self.tick_robot)
        self.timer_qr = self.create_timer(7.0, self.tick_qr)

    def tick_robot(self):
        # toggle one robot at a time
        if not self.robot_pubs:
            return
        name, topic, pub = self.robot_pubs[self.i % len(self.robot_pubs)]
        cur = self.robot_states[name]
        new = not cur
        self.robot_states[name] = new
        msg = Bool(data=new)
        pub.publish(msg)
        self.get_logger().info(f"{name} returned={new} -> {topic}")
        self.i += 1

    def tick_qr(self):
        ts = time.strftime('%H%M%S')
        msg = String(data=f"QR-{ts}")
        self.qr_pub.publish(msg)
        self.get_logger().info(f"qr: {msg.data} -> {self.qr_topic}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default=None)
    args = parser.parse_args()

    config = {}
    if args.config:
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f) or {}

    rclpy.init()
    node = MockNode(config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

