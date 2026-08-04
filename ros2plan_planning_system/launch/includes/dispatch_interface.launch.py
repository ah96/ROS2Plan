from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='ros2plan_plan_dispatcher'),
        DeclareLaunchArgument('knowledge_base', default_value='ros2plan_knowledge_base'),
        # The parsing interface publishes under a relative name, which resolves to
        # /complete_plan in the root namespace. Subscribe to the same relative name.
        DeclareLaunchArgument('plan_topic', default_value='complete_plan'),
        DeclareLaunchArgument('action_dispatch_topic', default_value='action_dispatch'),
        DeclareLaunchArgument('action_feedback_topic', default_value='action_feedback'),
        DeclareLaunchArgument('display_edge_type', default_value='false'),
        DeclareLaunchArgument('timeout_actions', default_value='false'),
        DeclareLaunchArgument('action_timeout_fraction', default_value='0.0'),

        Node(
            package='ros2plan_planning_system',
            executable='pddl_esterel_plan_dispatcher',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'knowledge_base': LaunchConfiguration('knowledge_base'),
                'plan_topic': LaunchConfiguration('plan_topic'),
                'action_dispatch_topic': LaunchConfiguration('action_dispatch_topic'),
                'action_feedback_topic': LaunchConfiguration('action_feedback_topic'),
                'display_edge_type': LaunchConfiguration('display_edge_type'),
                'timeout_actions': LaunchConfiguration('timeout_actions'),
                'action_timeout_fraction': LaunchConfiguration('action_timeout_fraction'),
            }],
        ),
    ])
