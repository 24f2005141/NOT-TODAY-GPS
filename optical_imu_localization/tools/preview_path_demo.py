"""Open the live-graph demonstration without requiring a ROS 2 installation."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--save', type=Path, help='Write the live demonstration as an animated GIF.')
    return parser.parse_args()


arguments = parse_arguments()
if arguments.save is not None:
    matplotlib.use('Agg')

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt


PATH = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (2.0, 2.0)]


def main() -> None:
    figure, (path_axis, trace_axis) = plt.subplots(1, 2, figsize=(12, 5))
    figure.canvas.manager.set_window_title('HereFlow live positioning — demo')

    path_line, = path_axis.plot([], [], '-o', color='#1f77b4', label='Live path')
    current_marker, = path_axis.plot([], [], 'o', color='#d62728', label='Current position')
    path_axis.plot([0.0], [0.0], 's', color='#2ca02c', label='Start (0, 0)')
    path_axis.set_title('Live X-Y path')
    path_axis.set_xlabel('X position (m): right')
    path_axis.set_ylabel('Y position (m): forward')
    path_axis.set_xlim(-0.5, 2.5)
    path_axis.set_ylim(-0.5, 2.5)
    path_axis.set_aspect('equal', adjustable='box')
    path_axis.grid(True, alpha=0.3)
    path_axis.legend(loc='best')

    x_trace, = trace_axis.plot([], [], color='#1f77b4', label='X position')
    y_trace, = trace_axis.plot([], [], color='#ff7f0e', label='Y position')
    trace_axis.set_title('Live coordinate change')
    trace_axis.set_xlabel('Elapsed time (s)')
    trace_axis.set_ylabel('Position (m)')
    trace_axis.set_xlim(-0.25, 4.25)
    trace_axis.set_ylim(-0.5, 2.5)
    trace_axis.grid(True, alpha=0.3)
    trace_axis.legend(loc='best')
    figure.tight_layout()

    def update(step_index: int):
        points = PATH[:step_index + 1]
        x_values, y_values = zip(*points)
        times = list(range(step_index + 1))
        path_line.set_data(x_values, y_values)
        current_marker.set_data([x_values[-1]], [y_values[-1]])
        x_trace.set_data(times, x_values)
        y_trace.set_data(times, y_values)
        return path_line, current_marker, x_trace, y_trace

    animation = FuncAnimation(figure, update, frames=len(PATH), interval=1000, repeat=False, blit=False)
    if arguments.save is not None:
        arguments.save.parent.mkdir(parents=True, exist_ok=True)
        animation.save(arguments.save, writer='pillow', fps=1)
        print(f'Animated preview saved to {arguments.save}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
