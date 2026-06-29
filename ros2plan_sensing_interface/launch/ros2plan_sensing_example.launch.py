from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_si = FindPackageShare('ros2plan_sensing_interface')

    return LaunchDescription([
        DeclareLaunchArgument('main_rate', default_value='10'),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([pkg_si, 'example.yaml']),
            description='Path to the sensing interface YAML config file'),

        Node(
            package='ros2plan_sensing_interface',
            executable='sensing_interface',
            name='ros2plan_sensing_interface',
            output='screen',
            parameters=[{
                'main_rate': LaunchConfiguration('main_rate'),
                'config_path': LaunchConfiguration('config_path'),
            }],
        ),
    ])
