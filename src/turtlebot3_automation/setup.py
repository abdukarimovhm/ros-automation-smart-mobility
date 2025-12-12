from setuptools import setup

package_name = 'turtlebot3_automation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        'turtlebot3_automation',
        'turtlebot3_automation.setup_automation',
        'turtlebot3_automation.maintenance',
        'turtlebot3_automation.navigation',
        'turtlebot3_automation.perception',
        'turtlebot3_automation.custom_features',
        'turtlebot3_automation.utils',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', [
            'launch/automation_bringup.launch.py',
            'launch/navigation_only.launch.py',
            'launch/object_detection.launch.py',
            'launch/maintenance.launch.py',
            'launch/qr_follow.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/navigation_params.yaml',
            'config/object_detection.yaml',
            'config/maintenance.yaml',
            'config/qr_follow.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Baratov Sokhibjon',
    maintainer_email='example@example.com',
    description='Automation toolkit for TurtleBot3 on ROS2 Humble spanning setup, maintenance, navigation, and perception.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automation_cli = turtlebot3_automation.cli:main',
            'maintenance_monitor = turtlebot3_automation.maintenance.monitor_node:main',
            'yolo_detector = turtlebot3_automation.perception.object_detection_node:main',
            'qr_follow = turtlebot3_automation.custom_features.qr_follow_node:main',
            'navigation_manager = turtlebot3_automation.navigation.autonomy_manager:main',
        ],
    },
)
