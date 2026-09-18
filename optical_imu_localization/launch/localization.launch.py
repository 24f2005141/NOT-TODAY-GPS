from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = get_package_share_directory('optical_imu_localization') + '/config/localization.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_bitrate', default_value='1000000'),
        DeclareLaunchArgument('hereflow_node_id', default_value='0'),
        Node(
            package='optical_imu_localization',
            executable='optical_imu_localizer',
            name='hereflow_localizer',
            output='screen',
            parameters=[config_file, {
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_z': LaunchConfiguration('initial_z'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'can_interface': LaunchConfiguration('can_interface'),
                'can_bitrate': LaunchConfiguration('can_bitrate'),
                'hereflow_node_id': LaunchConfiguration('hereflow_node_id'),
            }],
        ),
    ])
