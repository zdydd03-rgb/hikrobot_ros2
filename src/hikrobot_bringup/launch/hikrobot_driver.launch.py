import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    source_config = os.path.expanduser(
        '~/hikrobot_ros2/src/hikrobot_bringup/config/'
        'hikrobot_driver.yaml'
    )

    config_file = LaunchConfiguration('config_file')

    driver = LifecycleNode(
        package='hikrobot_driver',
        executable='hikrobot_driver_node',
        name='hikrobot_driver',
        namespace='',
        output='screen',
        parameters=[config_file],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=source_config,
            description='Hikrobot Driver parameter YAML',
        ),
        driver,
    ])
