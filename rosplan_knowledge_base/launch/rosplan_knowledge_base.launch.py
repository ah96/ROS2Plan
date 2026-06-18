from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ps = FindPackageShare('rosplan_planning_system')

    return LaunchDescription([
        DeclareLaunchArgument(
            'domain_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'domain.pddl']),
            description='Path to the PDDL domain file'),
        DeclareLaunchArgument(
            'use_unknowns',
            default_value='false',
            description='Enable conditional planning with unknown predicates'),
        DeclareLaunchArgument(
            'kb_rate',
            default_value='100.0',
            description='Rate (Hz) for timed initial literals'),

        Node(
            package='rosplan_knowledge_base',
            executable='knowledgeBase',
            name='rosplan_knowledge_base',
            output='screen',
            parameters=[{
                'domain_path': LaunchConfiguration('domain_path'),
                'use_unknowns': LaunchConfiguration('use_unknowns'),
                'kb_rate': LaunchConfiguration('kb_rate'),
            }],
        ),
    ])
