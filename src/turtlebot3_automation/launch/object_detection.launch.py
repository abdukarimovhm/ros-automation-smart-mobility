from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('turtlebot3_automation')
    detection_config = PathJoinSubstitution([package_share, 'config', 'object_detection.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = LaunchConfiguration('camera_topic')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/color/image_raw'),
    ]

    detector = Node(
        package='turtlebot3_automation',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[detection_config, {'use_sim_time': use_sim_time, 'camera_topic': camera_topic}],
        output='screen',
    )

    return LaunchDescription(declarations + [detector])
