from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ps = FindPackageShare('ros2plan_planning_system')

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='ros2plan_planner_interface'),
        DeclareLaunchArgument('use_problem_topic', default_value='true'),
        # The problem interface publishes under a relative name, which resolves to
        # /problem_instance in the root namespace. Subscribe to the same relative name.
        DeclareLaunchArgument('problem_topic', default_value='problem_instance'),
        DeclareLaunchArgument('planner_topic', default_value='planner_output'),
        DeclareLaunchArgument(
            'domain_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'domain.pddl'])),
        DeclareLaunchArgument(
            'problem_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'problem.pddl'])),
        DeclareLaunchArgument(
            'data_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', ''])),
        DeclareLaunchArgument(
            'planner_command',
            default_value=[
                'timeout 10 ',
                PathJoinSubstitution([pkg_ps, 'common', 'bin', 'popf']),
                ' DOMAIN PROBLEM',
            ]),
        DeclareLaunchArgument('planner_interface', default_value='popf_planner_interface'),

        Node(
            package='ros2plan_planning_system',
            executable=LaunchConfiguration('planner_interface'),
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'use_problem_topic': LaunchConfiguration('use_problem_topic'),
                'problem_topic': LaunchConfiguration('problem_topic'),
                'planner_topic': LaunchConfiguration('planner_topic'),
                'domain_path': LaunchConfiguration('domain_path'),
                'problem_path': LaunchConfiguration('problem_path'),
                'data_path': LaunchConfiguration('data_path'),
                'planner_command': LaunchConfiguration('planner_command'),
            }],
        ),
    ])
