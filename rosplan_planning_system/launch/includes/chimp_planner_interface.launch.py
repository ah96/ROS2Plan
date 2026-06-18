from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ps = FindPackageShare('rosplan_planning_system')
    pkg_deps = FindPackageShare('rosplan_dependencies')

    common_path_default = PathJoinSubstitution(
        [pkg_ps, 'test', 'hierarchical_domains', 'chimp', 'restaurant'])

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='rosplan_planner_interface'),
        DeclareLaunchArgument('use_problem_topic', default_value='true'),
        DeclareLaunchArgument(
            'problem_topic',
            default_value='/rosplan_problem_interface/problem_instance'),
        DeclareLaunchArgument('planner_topic', default_value='planner_output'),
        DeclareLaunchArgument('common_path', default_value=common_path_default),
        DeclareLaunchArgument(
            'domain_path',
            default_value=PathJoinSubstitution(
                [LaunchConfiguration('common_path'), 'race_waiter_domain.ddl'])),
        DeclareLaunchArgument(
            'problem_path',
            default_value=PathJoinSubstitution(
                [LaunchConfiguration('common_path'), 'generated_problem.pdl'])),
        DeclareLaunchArgument(
            'data_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'hierarchical_domains', 'chimp', 'restaurant', ''])),
        DeclareLaunchArgument(
            'planner_command',
            default_value=[
                'timeout 10 ',
                PathJoinSubstitution([pkg_deps, 'chimp', 'gradlew']),
                ' run -p ',
                PathJoinSubstitution([pkg_deps, 'chimp']),
                ' -Dexec.args="DOMAIN PROBLEM -o=OUTPUT"',
            ]),
        DeclareLaunchArgument('planner_interface', default_value='chimp_planner_interface'),

        Node(
            package='rosplan_planning_system',
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
