from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='ros2plan_parsing_interface'),
        DeclareLaunchArgument('knowledge_base', default_value='ros2plan_knowledge_base'),
        DeclareLaunchArgument('epsilon_time', default_value='0.01'),
        # The planner interface publishes under a relative name, which resolves to
        # /planner_output in the root namespace. Subscribe to the same relative name.
        DeclareLaunchArgument('planner_topic', default_value='planner_output'),
        DeclareLaunchArgument('plan_topic', default_value='complete_plan'),

        Node(
            package='ros2plan_planning_system',
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
