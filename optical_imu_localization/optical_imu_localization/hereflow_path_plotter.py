#!/usr/bin/env python3
"""Show live local odometry as a side-by-side trajectory and position plot."""

from __future__ import annotations

from collections import deque
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class HereFlowPathPlotter(Node):
    """Plot the latest odometry path and its X/Y coordinates in real time."""

    def __init__(self) -> None:
        super().__init__('hereflow_path_plotter')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_history_points', 2_000)
        self.declare_parameter('refresh_rate_hz', 20.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.0)

        history_size = int(self.get_parameter('max_history_points').value)
        refresh_rate = float(self.get_parameter('refresh_rate_hz').value)
        if history_size < 2 or refresh_rate <= 0.0:
            raise ValueError('max_history_points must be at least 2 and refresh_rate_hz must be positive')

        self.x_values: deque[float] = deque(maxlen=history_size)
        self.y_values: deque[float] = deque(maxlen=history_size)
        self.z_values: deque[float] = deque(maxlen=history_size)
        self.times: deque[float] = deque(maxlen=history_size)
        self.start_time: Optional[float] = None
        self.new_data = True

        # The explicit zero initial point makes the coordinate origin visible
        # before the first live HereFlow odometry update arrives.
        self.add_sample(
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
            float(self.get_parameter('start_z').value),
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self.odom_callback, 50
        )
        self.initialize_plot()
        self.create_timer(1.0 / refresh_rate, self.refresh_plot)

    def initialize_plot(self) -> None:
        """Create the desktop chart window lazily so ROS data remains real-time."""
        try:
            import matplotlib.pyplot as plt
        except ImportError as error:
            raise RuntimeError(
                'Matplotlib is required for the live plot. Install it with: python3 -m pip install matplotlib'
            ) from error

        self.plt = plt
        plt.ion()
        self.figure, (self.path_axis, self.trace_axis) = plt.subplots(1, 2, figsize=(12, 5))
        self.figure.canvas.manager.set_window_title('HereFlow live positioning')

        self.path_line, = self.path_axis.plot([], [], '-', color='#1f77b4', label='Path')
        self.path_marker, = self.path_axis.plot([], [], 'o', color='#d62728', label='Current position')
        self.path_axis.plot([0.0], [0.0], 's', color='#2ca02c', label='Start (0, 0)')
        self.path_axis.set_title('Live X-Y path')
        self.path_axis.set_xlabel('X position (m)')
        self.path_axis.set_ylabel('Y position (m)')
        self.path_axis.set_aspect('equal', adjustable='box')
        self.path_axis.grid(True, alpha=0.3)
        self.path_axis.legend(loc='best')

        self.x_trace, = self.trace_axis.plot([], [], color='#1f77b4', label='X position')
        self.y_trace, = self.trace_axis.plot([], [], color='#ff7f0e', label='Y position')
        self.trace_axis.set_title('Live coordinate change')
        self.trace_axis.set_xlabel('Elapsed time (s)')
        self.trace_axis.set_ylabel('Position (m)')
        self.trace_axis.grid(True, alpha=0.3)
        self.trace_axis.legend(loc='best')
        self.figure.tight_layout()
        plt.show(block=False)

    def add_sample(self, x: float, y: float, z: float) -> None:
        """Store an incoming position while retaining a bounded graph history."""
        timestamp = time.monotonic()
        if self.start_time is None:
            self.start_time = timestamp
        self.times.append(timestamp - self.start_time)
        self.x_values.append(x)
        self.y_values.append(y)
        self.z_values.append(z)
        self.new_data = True

    def odom_callback(self, msg: Odometry) -> None:
        """Receive the real-time position from the HereFlow localizer."""
        position = msg.pose.pose.position
        self.add_sample(position.x, position.y, position.z)

    @staticmethod
    def padded_limits(values: list[float]) -> tuple[float, float]:
        """Return visible axis limits, including a useful margin for a flat path."""
        lower, upper = min(values), max(values)
        padding = max((upper - lower) * 0.15, 0.25)
        return lower - padding, upper + padding

    def refresh_plot(self) -> None:
        """Redraw both graphs only after ROS receives an updated coordinate."""
        if not self.plt.fignum_exists(self.figure.number):
            self.get_logger().info('Plot window closed; shutting down visualizer')
            rclpy.shutdown()
            return
        if not self.new_data:
            self.plt.pause(0.001)
            return

        x_values, y_values, times = list(self.x_values), list(self.y_values), list(self.times)
        self.path_line.set_data(x_values, y_values)
        self.path_marker.set_data([x_values[-1]], [y_values[-1]])
        self.path_axis.set_xlim(*self.padded_limits(x_values))
        self.path_axis.set_ylim(*self.padded_limits(y_values))

        self.x_trace.set_data(times, x_values)
        self.y_trace.set_data(times, y_values)
        self.trace_axis.set_xlim(*self.padded_limits(times))
        self.trace_axis.set_ylim(*self.padded_limits(x_values + y_values))
        self.figure.canvas.draw_idle()
        self.plt.pause(0.001)
        self.new_data = False

    def destroy_node(self) -> bool:
        self.plt.close(self.figure)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HereFlowPathPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
