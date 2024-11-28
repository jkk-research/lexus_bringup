from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction

from launch.substitutions import LaunchConfiguration, FindExecutable
# from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """
    NAMESPACE = "lexus3"

    pkg_dir = get_package_share_directory('lexus_bringup')
    params_file_path = Path(pkg_dir) / 'launch' / 'drivers' / 'ouster_config_b.yaml'
    merger_params_file_path = Path(pkg_dir) / 'launch' / 'drivers' / 'ouster_config_comp_b_two_lidars.yaml'


    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(params_file_path),
        description='Ouster driver parameters.'
    )

    merger_params_file = LaunchConfiguration('merger_params_file')
    merger_params_file_arg = DeclareLaunchArgument(
        'merger_params_file',
        default_value=str(merger_params_file_path),
        description='Ouster PCL Merger parameters.'
    )

    ouster_ns = LaunchConfiguration(NAMESPACE)
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='lexus3')


    os_left_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_driver',
        namespace='lexus3/os_left',
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_left_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace='lexus3/os_left',
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_right_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_driver',
        namespace='lexus3/os_right',
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_right_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace='lexus3/os_right',
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # os_center_sensor = ComposableNode(
    #     package='ouster_ros',
    #     plugin='ouster_ros::OusterSensor',
    #     name='os_driver',
    #     namespace='lexus3/os_center',
    #     parameters=[params_file],
    #     extra_arguments=[{'use_intra_process_comms': True}],
    # )

    # os_center_cloud = ComposableNode(
    #     package='ouster_ros',
    #     plugin='ouster_ros::OusterCloud',
    #     name='os_cloud',
    #     namespace='lexus3/os_center',
    #     parameters=[params_file],
    #     extra_arguments=[{'use_intra_process_comms': True}],
    # )

    os_pcl_merger = ComposableNode(
        package='lexus_bringup',
        plugin='merger::OusterPCLMerger',
        # executable from `rclcpp_components_register_node` (CMakeLists.txt)
        name='os_pcl_merger_node',
        namespace='lexus3',
        parameters=[merger_params_file],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_container = ComposableNodeContainer(
        name='os_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            os_left_sensor,
            os_right_sensor,
            os_left_cloud,
            os_right_cloud,
            # os_center_sensor,
            # os_center_cloud,
            os_pcl_merger,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    def invoke_lifecycle_cmd(node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set /', node_name, ' ', verb]],
            shell=True
        )

    sensor_left_configure_cmd = invoke_lifecycle_cmd('lexus3/os_left/os_driver', 'configure')
    sensor_left_activate_cmd = invoke_lifecycle_cmd('lexus3/os_left/os_driver', 'activate')
    sensor_right_configure_cmd = invoke_lifecycle_cmd('lexus3/os_right/os_driver', 'configure')
    sensor_right_activate_cmd = invoke_lifecycle_cmd('lexus3/os_right/os_driver', 'activate')
    # sensor_center_configure_cmd = invoke_lifecycle_cmd('lexus3/os_center/os_driver', 'configure')
    # sensor_center_activate_cmd = invoke_lifecycle_cmd('lexus3/os_center/os_driver', 'activate')
    

    return LaunchDescription([
        params_file_arg,
        merger_params_file_arg,
        ouster_ns_arg,
        os_container,
        TimerAction(period=4.0, actions=[sensor_left_configure_cmd]),
        TimerAction(period=8.0, actions=[sensor_left_activate_cmd]),
        TimerAction(period=12.0, actions=[sensor_right_configure_cmd]),
        TimerAction(period=16.0, actions=[sensor_right_activate_cmd]),
        # TimerAction(period=20.0, actions=[sensor_center_configure_cmd]),
        # TimerAction(period=24.0, actions=[sensor_center_activate_cmd]),
    ])
