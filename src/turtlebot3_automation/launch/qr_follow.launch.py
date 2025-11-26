from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('turtlebot3_automation')
    qr_config = PathJoinSubstitution([package_share, 'config', 'qr_follow.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    target = LaunchConfiguration('target')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('target', default_value='FOLLOW_ME'),
    ]

    node = Node(
        package='turtlebot3_automation',
        executable='qr_follow',
        name='qr_follow',
        parameters=[qr_config, {'use_sim_time': use_sim_time, 'target': target}],
        output='screen',
    )

    return LaunchDescription(declarations + [node])
