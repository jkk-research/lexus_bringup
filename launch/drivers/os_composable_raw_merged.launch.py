from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration, FindExecutable, TextSubstitution
from os.path import join


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """

    ouster_namespace_arg = DeclareLaunchArgument(
        'ouster_ns',
        default_value='lexus3',
        description='Namespace for the Ouster sensor node')
    ouster_left_full_namespace_arg = DeclareLaunchArgument(
        'ouster_full_ns_left',
        default_value='lexus3/os_left',
        description='Complete namespace for the left Ouster sensor node')
    ouster_right_full_namespace_arg = DeclareLaunchArgument(
        'ouster_full_ns_right',
        default_value='lexus3/os_right',
        description='Complete namespace for the right Ouster sensor node')
    ouster_center_full_namespace_arg = DeclareLaunchArgument(
        'ouster_full_ns_center',
        default_value='lexus3/os_center',
        description='Complete namespace for the center Ouster sensor node')
    merged_pc_topic_name_arg = DeclareLaunchArgument(
        'merged_pc_topic_name',
        default_value='sensing/lidar/concatenated/pointcloud',
        description='Topic name of the merged pointcloud')
    merged_pc_frequency_arg = DeclareLaunchArgument(
        'merged_pc_frequency',
        default_value='10.0',
        description='Frequency of the merged pointcloud')
    ouster_left_param_arg = DeclareLaunchArgument(
        'os_left_params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'config',
            'lidar',
            'ouster_config_left.yaml'
        )
    )
    ouster_right_param_arg = DeclareLaunchArgument(
        'os_right_params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'config',
            'lidar',
            'ouster_config_right.yaml'
        )
    )
    ouster_center_param_arg = DeclareLaunchArgument(
        'os_center_params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'config',
            'lidar',
            'ouster_config_center.yaml'
        )
    )
    merger_param_arg = DeclareLaunchArgument(
        'merger_params_file',
        default_value=join(
            get_package_share_directory('lexus_bringup'),
            'config',
            'lidar',
            'ouster_config_comp.yaml'
        )
    )

    os_left_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_driver',
        namespace=LaunchConfiguration('ouster_full_ns_left'),
        parameters=[LaunchConfiguration('os_left_params_file')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_left_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=LaunchConfiguration('ouster_full_ns_left'),
        parameters=[LaunchConfiguration('os_left_params_file')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_right_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_driver',
        namespace=LaunchConfiguration('ouster_full_ns_right'),
        parameters=[LaunchConfiguration('os_right_params_file')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_right_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=LaunchConfiguration('ouster_full_ns_right'),
        parameters=[LaunchConfiguration('os_right_params_file')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_center_sensor = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterSensor',
        name='os_driver',
        namespace=LaunchConfiguration('ouster_full_ns_center'),
        parameters=[LaunchConfiguration('os_center_params_file')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    os_center_cloud = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace=LaunchConfiguration('ouster_full_ns_center'),
        parameters=[LaunchConfiguration('os_center_params_file')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    os_pcl_merger = ComposableNode(
        package='lexus_bringup',
        plugin='merger::OusterPCLMerger',
        # executable from `rclcpp_components_register_node` (CMakeLists.txt)
        name='os_pcl_merger_node',
        namespace=LaunchConfiguration('ouster_ns'),
        parameters=[
            LaunchConfiguration('merger_params_file'),
            {
                'topics': [
                    [LaunchConfiguration('ouster_full_ns_left'),   '/points'],
                    [LaunchConfiguration('ouster_full_ns_right'),  '/points'],
                    [LaunchConfiguration('ouster_full_ns_center'), '/points']
                ],
                'target_frame'  : [LaunchConfiguration('ouster_full_ns_left'), '/os_center_a'],
                'pub_topic_name': LaunchConfiguration('merged_pc_topic_name'),
                'merger_freq'   : LaunchConfiguration('merged_pc_frequency'),
                'crop_box': {
                    'min_x'   : -1.5,  # Points with X < "val" will be excluded. (half of the vehicle's length backward)
                    'min_y'   : -0.95, # Points with Y < "val" will be excluded. (half of the vehicle's width to the right)
                    'min_z'   : -2.43, # Points with Z < "val" will be excluded. (slightly below the ground to account for any mounting height)
                    'max_x'   : 3.50,  # Points with X > "val" will be excluded. (half of the vehicle's length forward)
                    'max_y'   : 0.95,  # Points with Y > "val" will be excluded. (half of the vehicle's width to the left)
                    'max_z'   : 0.20,  # Points with Z > "val" will be excluded. (slightly above the vehicle to account for the roof and mounting height)
                    'negative': True,  # Set to true to invert the CropBox filter, filtering points inside the defined box.
                    'apply'   : True   # Set to true to apply the filter (on / off switch)
                }
            }
        ],
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
            os_center_sensor,
            os_center_cloud,
            os_pcl_merger,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    def invoke_lifecycle_cmd(namespace_arg_name, node_name, verb):
        ros2_exec = FindExecutable(name='ros2')
        return ExecuteProcess(
            cmd=[[ros2_exec, ' lifecycle set /', LaunchConfiguration(namespace_arg_name), '/', node_name, ' ', verb]],
            shell=True
        )

    return LaunchDescription([
        ouster_namespace_arg,
        ouster_left_full_namespace_arg,
        ouster_right_full_namespace_arg,
        ouster_center_full_namespace_arg,
        merged_pc_topic_name_arg,
        merged_pc_frequency_arg,
        ouster_left_param_arg,
        ouster_right_param_arg,
        ouster_center_param_arg,
        merger_param_arg,
        os_container,

        TimerAction(period=4.0, actions=[invoke_lifecycle_cmd('ouster_full_ns_left', 'os_driver', 'configure')]),
        TimerAction(period=8.0, actions=[invoke_lifecycle_cmd('ouster_full_ns_left', 'os_driver', 'activate')]),
        TimerAction(period=12.0, actions=[invoke_lifecycle_cmd('ouster_full_ns_right', 'os_driver', 'configure')]),
        TimerAction(period=16.0, actions=[invoke_lifecycle_cmd('ouster_full_ns_right', 'os_driver', 'activate')]),
        TimerAction(period=20.0, actions=[invoke_lifecycle_cmd('ouster_full_ns_center', 'os_driver', 'configure')]),
        TimerAction(period=24.0, actions=[invoke_lifecycle_cmd('ouster_full_ns_center', 'os_driver', 'activate')]),
    ])
