from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('turtlebot3_automation')
    nav2_share = FindPackageShare('nav2_bringup')
    slam_share = FindPackageShare('slam_toolbox')

    navigation_config = PathJoinSubstitution([package_share, 'config', 'navigation_params.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map_yaml')
    nav2_params = LaunchConfiguration('nav2_params')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='slam'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([nav2_share, 'params', 'nav2_params.yaml']),
        ),
    ]

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': PythonExpression(["'True' if '", mode, "' == 'slam' else 'False'"]),
            'map': map_yaml,
            'params_file': nav2_params,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, 'launch', 'online_async_launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation_manager = Node(
        package='turtlebot3_automation',
        executable='navigation_manager',
        name='navigation_manager',
        parameters=[navigation_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(declarations + [nav2_launch, slam_launch, navigation_manager])
