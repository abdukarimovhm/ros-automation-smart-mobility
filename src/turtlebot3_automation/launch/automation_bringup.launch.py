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
    maintenance_config = PathJoinSubstitution([package_share, 'config', 'maintenance.yaml'])
    detection_config = PathJoinSubstitution([package_share, 'config', 'object_detection.yaml'])
    qr_config = PathJoinSubstitution([package_share, 'config', 'qr_follow.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    mode = LaunchConfiguration('mode')
    map_yaml = LaunchConfiguration('map_yaml')
    nav2_params = LaunchConfiguration('nav2_params')
    start_detection = LaunchConfiguration('start_detection')
    start_qr_follow = LaunchConfiguration('start_qr_follow')
    start_maintenance = LaunchConfiguration('start_maintenance')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='slam', description='slam or map'),
        DeclareLaunchArgument('map_yaml', default_value='', description='map yaml when mode=map'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([nav2_share, 'params', 'nav2_params.yaml']),
        ),
        DeclareLaunchArgument('start_detection', default_value='true'),
        DeclareLaunchArgument('start_qr_follow', default_value='false'),
        DeclareLaunchArgument('start_maintenance', default_value='true'),
    ]

    actions = []

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
    actions.append(nav2_launch)

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_share, 'launch', 'online_async_launch.py'])
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    actions.append(slam_launch)

    navigation_manager = Node(
        package='turtlebot3_automation',
        executable='navigation_manager',
        name='navigation_manager',
        parameters=[navigation_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    actions.append(navigation_manager)

    maintenance_node = Node(
        condition=IfCondition(start_maintenance),
        package='turtlebot3_automation',
        executable='maintenance_monitor',
        name='maintenance_monitor',
        parameters=[maintenance_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    actions.append(maintenance_node)

    detection_node = Node(
        condition=IfCondition(start_detection),
        package='turtlebot3_automation',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[detection_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    actions.append(detection_node)

    qr_follow_node = Node(
        condition=IfCondition(start_qr_follow),
        package='turtlebot3_automation',
        executable='qr_follow',
        name='qr_follow',
        parameters=[qr_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    actions.append(qr_follow_node)

    return LaunchDescription(declarations + actions)
