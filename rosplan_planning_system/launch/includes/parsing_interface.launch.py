from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='rosplan_parsing_interface'),
        DeclareLaunchArgument('knowledge_base', default_value='rosplan_knowledge_base'),
        DeclareLaunchArgument('epsilon_time', default_value='0.01'),
        DeclareLaunchArgument(
            'planner_topic',
            default_value='/rosplan_planner_interface/planner_output'),
        DeclareLaunchArgument('plan_topic', default_value='complete_plan'),

        Node(
            package='rosplan_planning_system',
            executable='pddl_esterel_plan_parser',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'knowledge_base': LaunchConfiguration('knowledge_base'),
                'epsilon_time': LaunchConfiguration('epsilon_time'),
                'planner_topic': LaunchConfiguration('planner_topic'),
                'plan_topic': LaunchConfiguration('plan_topic'),
            }],
        ),
    ])
