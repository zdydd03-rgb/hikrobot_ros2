from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hikrobot_bringup'),
        'config', 'hikrobot_driver.yaml')

    driver = LifecycleNode(
        package='hikrobot_driver',
        executable='hikrobot_driver_node',
        name='hikrobot_driver',
        output='screen',
        parameters=[config],
    )

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(driver),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    activate_when_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(driver),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )

    return LaunchDescription([driver, activate_when_inactive, configure])
