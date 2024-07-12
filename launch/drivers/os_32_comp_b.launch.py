from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction

from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """
    NODE_NAME = "os_comp"
    NAMESPACE = "/lexus3"

    pkg_dir = get_package_share_directory('lexus_bringup')
    default_params_file = \
        Path(pkg_dir) / 'launch' / 'drivers' / 'ouster_config_b.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    # TODO: untangle namespaces (ouster_ns, /lexus3), any needed for launch, conventions etc.
    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster')
    
    # TODO: check if merger config is needed at all here


    os_left = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_left',
        namespace='',
        parameters=[params_file],
        remappings=[('/points', NAMESPACE + "/os_left" + '/points')],
        # extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_right = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_right',
        namespace='',
        parameters=[params_file],
        remappings=[('/points', NAMESPACE + "/os_right" + '/points')],
        # extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_left,
            os_right
        ],
        output='screen',
    )

    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set /', node_name, ' ', verb]],
            shell=True)

    sensor_left_configure_cmd = invoke_lifecycle_cmd('os_left', 'configure')
    sensor_left_activate_cmd = invoke_lifecycle_cmd('os_left', 'activate')
    sensor_right_configure_cmd = invoke_lifecycle_cmd('os_right', 'configure')
    sensor_right_activate_cmd = invoke_lifecycle_cmd('os_right', 'activate')

    return LaunchDescription([
        params_file_arg,
        ouster_ns_arg,
        os_container,
        # PushRosNamespace('lexus3/os_left'),
        TimerAction(period=2.0, actions=[sensor_left_configure_cmd]),
        TimerAction(period=10.0, actions=[sensor_left_activate_cmd]),
        # PushRosNamespace('lexus3/os_right'),
        TimerAction(period=2.0, actions=[sensor_right_configure_cmd]),
        TimerAction(period=10.0, actions=[sensor_right_activate_cmd]),
        # static_tf  # TODO: add later
    ])

