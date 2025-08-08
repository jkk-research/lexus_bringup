# Copyright 2023 Ouster, Inc.

from launch import LaunchDescription
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from os.path import join


def generate_launch_description():
    ouster_namespace_arg = DeclareLaunchArgument(
        'ouster_ns',
        default_value='lexus3/os_left',
        description='Namespace for the Ouster sensor node')

    params_file_arg = DeclareLaunchArgument('params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'launch',
            'drivers',
            'ouster_config_b.yaml'
        ),
        description='Name or path to the parameter file to use.'
    )

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name="os_driver",
        namespace=LaunchConfiguration('ouster_ns'),
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='inactive',
            entities=[
                LogInfo(msg="os_driver activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    return LaunchDescription([
        ouster_namespace_arg,
        params_file_arg,

        os_driver,
        sensor_configure_event,
        sensor_activate_event
    ])
 