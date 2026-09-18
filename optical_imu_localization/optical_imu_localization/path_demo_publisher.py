#!/usr/bin/env python3
"""Publish a repeatable X/Y odometry path for validating the live plot."""

from __future__ import annotations

from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PathDemoPublisher(Node):
    """Publish: origin, two rightward steps, then two forward steps."""

    def __init__(self) -> None:
        super().__init__('hereflow_path_demo')
        self.declare_parameter('step_m', 1.0)
        self.declare_parameter('step_period_s', 1.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        step_m = float(self.get_parameter('step_m').value)
        step_period_s = float(self.get_parameter('step_period_s').value)
        if step_m <= 0.0 or step_period_s <= 0.0:
            raise ValueError('step_m and step_period_s must be positive')

        # This test path ends at (+2, +2): two steps right on X, then two
        # forward steps on Y. It is independent of the live HereFlow driver.
        self.waypoints = [(0.0, 0.0), (step_m, 0.0), (2 * step_m, 0.0),
                          (2 * step_m, step_m), (2 * step_m, 2 * step_m)]
        self.index = 0
        self.publisher = self.create_publisher(Odometry, 'simulated_odom', 10)
        self.create_timer(step_period_s, self.publish_next_waypoint)
        self.publish_next_waypoint()

    def publish_next_waypoint(self) -> None:
        """Publish the next live-like sample, retaining the final coordinate."""
        x, y = self.waypoints[self.index]
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        message.child_frame_id = str(self.get_parameter('child_frame_id').value)
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        self.publisher.publish(message)
        if self.index < len(self.waypoints) - 1:
            self.index += 1


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PathDemoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
