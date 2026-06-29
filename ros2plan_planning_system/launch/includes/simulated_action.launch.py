from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pddl_action_name',
                              description='Name of the PDDL action this node handles'),
        DeclareLaunchArgument('action_duration', default_value='0.0'),
        DeclareLaunchArgument('action_duration_stddev', default_value='0.0'),
        DeclareLaunchArgument('action_probability', default_value='1.0'),
        DeclareLaunchArgument('knowledge_base', default_value='ros2plan_knowledge_base'),
        DeclareLaunchArgument(
            'action_dispatch_topic',
            default_value='/ros2plan_plan_dispatcher/action_dispatch'),
        DeclareLaunchArgument(
            'action_feedback_topic',
            default_value='/ros2plan_plan_dispatcher/action_feedback'),

        Node(
            package='ros2plan_planning_system',
            executable='simulatedAction',
            name=['ros2plan_interface_', LaunchConfiguration('pddl_action_name')],
            output='screen',
            parameters=[{
                'knowledge_base': LaunchConfiguration('knowledge_base'),
                'pddl_action_name': LaunchConfiguration('pddl_action_name'),
                'action_duration': LaunchConfiguration('action_duration'),
                'action_duration_stddev': LaunchConfiguration('action_duration_stddev'),
                'action_probability': LaunchConfiguration('action_probability'),
                'action_dispatch_topic': LaunchConfiguration('action_dispatch_topic'),
                'action_feedback_topic': LaunchConfiguration('action_feedback_topic'),
            }],
        ),
    ])
