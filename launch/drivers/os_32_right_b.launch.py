# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LifecycleNode
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
#from launch_ros.actions import SetRemap



def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    namespace_lexus = "/lexus3"
    node_id = 'os_right'
    lexus_ros_pkg_dir = get_package_share_directory('lexus_bringup')
    default_params_file = \
        Path(lexus_ros_pkg_dir) / 'launch' / 'drivers' / 'os_32_right_b.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument('ouster_ns', default_value='ouster')


    os_sensor_right = LifecycleNode(
        package='ouster_ros',
        executable='os_sensor',
        name='os_sensor_right',
        namespace=ouster_ns,
        parameters=[params_file],
        remappings=[
            ('/ouster/metadata', "/" + node_id + '/metadata'),   
            ('/ouster/imu_packets', "/" + node_id + '/imu_packets'),
            ('/ouster/lidar_packets', "/" + node_id + '/lidar_packets'),
            #('/ouster/os_sensor/transition_event', "/" + node_id + '/transition_event'),
        ],   

        output='screen',
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_sensor_right),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_sensor_right, goal_state='inactive',
            entities=[
                LogInfo(msg="os_sensor activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_sensor_right),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )

    # TODO: figure out why registering for on_shutdown event causes an exception
    # and error handling
    # shutdown_event = RegisterEventHandler(
    #     OnShutdown(
    #         on_shutdown=[
    #             EmitEvent(event=ChangeState(
    #               lifecycle_node_matcher=matches_node_name(node_name=F"/ouster/os_sensor"),
    #               transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
    #             )),
    #             LogInfo(msg="os_sensor node exiting..."),
    #         ],
    #     )
    # )

    os_cloud = Node(
        package='ouster_ros',
        executable='os_cloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[params_file],
        remappings=[
            ('/ouster/points', namespace_lexus + "/" + node_id + '/points'),   
            ('/ouster/imu', namespace_lexus + "/" + node_id + '/imu'),
            ('/ouster/metadata', "/" + node_id + '/metadata'),   
            ('/ouster/imu_packets', "/" + node_id + '/imu_packets'),
            ('/ouster/lidar_packets', "/" + node_id + '/lidar_packets'),
            #('/ouster/os_sensor/transition_event', "/" + node_id + '/transition_event'),
        ],   
        output='screen',
    )

    # os_image = Node(
    #     package='ouster_ros',
    #     executable='os_image',
    #     name='os_image',
    #     namespace=ouster_ns,
    #     parameters=[params_file],
    #     remappings=[
    #         ('/ouster/range_image',  node_id  + '/range_image'),
    #         ('/ouster/nearir_image',  node_id  + '/nearir_image'),
    #         ('/ouster/reflec_image',  node_id  + '/reflec_image'),
    #         ('/ouster/signal_image',  node_id  + '/signal_image'),
    #     ],
    #     output='screen',
    # )
    
    return launch.LaunchDescription([
        params_file_arg,
        ouster_ns_arg,
        os_sensor_right,
        os_cloud,
        # os_image,
        sensor_configure_event,
        sensor_activate_event,
        # shutdown_event
    ])
