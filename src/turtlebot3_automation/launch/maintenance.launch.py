from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('turtlebot3_automation')
    maintenance_config = PathJoinSubstitution([package_share, 'config', 'maintenance.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    monitor = Node(
        package='turtlebot3_automation',
        executable='maintenance_monitor',
        name='maintenance_monitor',
        parameters=[maintenance_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(declarations + [monitor])
