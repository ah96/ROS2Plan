from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ps = FindPackageShare('ros2plan_planning_system')
    includes = PathJoinSubstitution([pkg_ps, 'launch', 'includes'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'domain_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'domain.pddl'])),
        DeclareLaunchArgument('problem_path', default_value=''),
        DeclareLaunchArgument(
            'autom_gen_problem_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', 'problem.pddl'])),
        DeclareLaunchArgument(
            'data_path',
            default_value=PathJoinSubstitution(
                [pkg_ps, 'test', 'pddl', 'turtlebot', ''])),
        DeclareLaunchArgument('use_problem_topic', default_value='true'),
        DeclareLaunchArgument(
            'planner_command',
            default_value=[
                'timeout 10 ',
                PathJoinSubstitution([pkg_ps, 'common', 'bin', 'popf']),
                ' DOMAIN PROBLEM',
            ]),
        DeclareLaunchArgument('planner_interface', default_value='popf_planner_interface'),

        # Knowledge base
        Node(
            package='ros2plan_knowledge_base',
            executable='knowledgeBase',
            name='ros2plan_knowledge_base',
            output='screen',
            parameters=[{
                'domain_path': LaunchConfiguration('domain_path'),
                'problem_path': LaunchConfiguration('problem_path'),
                'use_unknowns': False,
            }],
        ),

        # Each include is wrapped in a scoped GroupAction. IncludeLaunchDescription does
        # not push a scope of its own, so without this the first `node_name` declared
        # (the problem interface's) sticks for every later include -- leaving all four
        # nodes sharing one name -- and the same leak applies to every other argument.

        # Problem generation
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([includes, 'problem_interface.launch.py'])),
                launch_arguments={
                    'knowledge_base': 'ros2plan_knowledge_base',
                    'domain_path': LaunchConfiguration('domain_path'),
                    'problem_path': LaunchConfiguration('autom_gen_problem_path'),
                    'problem_topic': 'problem_instance',
                }.items(),
            ),
        ], scoped=True),

        # Planner interface
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([includes, 'planner_interface.launch.py'])),
                launch_arguments={
                    'use_problem_topic': LaunchConfiguration('use_problem_topic'),
                    'problem_topic': 'problem_instance',
                    'planner_topic': 'planner_output',
                    'domain_path': LaunchConfiguration('domain_path'),
                    'problem_path': LaunchConfiguration('autom_gen_problem_path'),
                    'data_path': LaunchConfiguration('data_path'),
                    'planner_command': LaunchConfiguration('planner_command'),
                    'planner_interface': LaunchConfiguration('planner_interface'),
                }.items(),
            ),
        ], scoped=True),

        # Plan parsing
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([includes, 'parsing_interface.launch.py'])),
                launch_arguments={
                    'knowledge_base': 'ros2plan_knowledge_base',
                    'planner_topic': 'planner_output',
                    'plan_topic': 'complete_plan',
                }.items(),
            ),
        ], scoped=True),

        # Plan dispatch
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([includes, 'dispatch_interface.launch.py'])),
                launch_arguments={
                    'knowledge_base': 'ros2plan_knowledge_base',
                    'plan_topic': 'complete_plan',
                    'action_dispatch_topic': 'action_dispatch',
                    'action_feedback_topic': 'action_feedback',
                }.items(),
            ),
        ], scoped=True),
    ])
