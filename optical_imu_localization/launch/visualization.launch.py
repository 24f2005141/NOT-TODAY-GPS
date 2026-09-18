from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        Node(
            package='optical_imu_localization',
            executable='hereflow_path_plotter',
            name='hereflow_path_plotter',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'start_x': 0.0,
                'start_y': 0.0,
                'start_z': 0.0,
            }],
        ),
    ])
