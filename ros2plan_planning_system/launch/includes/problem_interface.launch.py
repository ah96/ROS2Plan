from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ps = FindPackageShare('ros2plan_planning_system')

    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='ros2plan_problem_interface'),
        DeclareLaunchArgument('knowledge_base', default_value='ros2plan_knowledge_base'),
        DeclareLaunchArgument(
            'domain_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'domain.pddl'])),
        DeclareLaunchArgument(
            'problem_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'problem.pddl'])),
        DeclareLaunchArgument('problem_topic', default_value='problem_instance'),
        DeclareLaunchArgument(
            'planning_language',
            default_value='PDDL',
            description='Planning language: PDDL, RDDL, or PPDDL'),

        Node(
            package='ros2plan_planning_system',
            executable='problemInterface',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'knowledge_base': LaunchConfiguration('knowledge_base'),
                'domain_path': LaunchConfiguration('domain_path'),
                'problem_path': LaunchConfiguration('problem_path'),
                'problem_topic': LaunchConfiguration('problem_topic'),
                'planning_language': LaunchConfiguration('planning_language'),
            }],
        ),
    ])
