from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='optical_imu_localization',
            executable='hereflow_path_demo',
            name='hereflow_path_demo',
            output='screen',
            parameters=[{'step_m': 1.0, 'step_period_s': 1.0}],
        ),
        Node(
            package='optical_imu_localization',
            executable='hereflow_path_plotter',
            name='hereflow_path_plotter',
            output='screen',
            parameters=[{
                'odom_topic': '/simulated_odom',
                'start_x': 0.0,
                'start_y': 0.0,
                'start_z': 0.0,
            }],
        ),
    ])
