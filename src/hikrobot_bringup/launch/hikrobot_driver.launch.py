import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('hikrobot_bringup'), 'config', 'hikrobot_driver.yaml'
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
            default_value=default_config,
            description='Hikrobot Driver parameter YAML',
        ),
        driver,
    ])
