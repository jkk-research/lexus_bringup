import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode


os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'


# Lexus common config
default_config_common = os.path.join(
    get_package_share_directory('lexus_bringup'),
    'config',
    'camera',
    'zed_default_common.yaml'
)

# ZED object detection configok
default_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'object_detection.yaml'
)

default_custom_object_detection_config_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'custom_object_detection.yaml'
)

# URDF/xacro
default_xacro_path = os.path.join(
    get_package_share_directory('zed_description'),
    'urdf',
    'zed_descr.urdf.xacro'
)


def parse_array_param(param: str):
    cleaned = param.replace('[', '').replace(']', '').replace(' ', '')
    if not cleaned:
        return []
    return cleaned.split(',')


def launch_setup(context, *args, **kwargs):
    actions = []

    node_log_type = LaunchConfiguration('node_log_type')

    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    node_name = LaunchConfiguration('node_name')
    container_name = LaunchConfiguration('container_name')

    config_path = LaunchConfiguration('config_path')
    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    object_detection_config_path = LaunchConfiguration('object_detection_config_path')
    custom_object_detection_config_path = LaunchConfiguration('custom_object_detection_config_path')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')

    xacro_path = LaunchConfiguration('xacro_path')

    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')

    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_ipc = LaunchConfiguration('enable_ipc')

    base_frame = LaunchConfiguration('base_frame')
    cam_pose = LaunchConfiguration('cam_pose')

    namespace_val = namespace.perform(context).strip('/')
    camera_name_val = camera_name.perform(context).strip('/')
    camera_model_val = camera_model.perform(context)
    node_name_val = node_name.perform(context)
    container_name_val = container_name.perform(context)
    node_log_type_val = node_log_type.perform(context)
    ros_params_override_path_val = ros_params_override_path.perform(context)

    if not camera_name_val:
        camera_name_val = camera_model_val

    if not namespace_val:
        namespace_val = camera_name_val

    if not container_name_val:
        container_name_val = 'zed_container'

    if node_log_type_val == 'both':
        node_output = 'screen'
    else:
        node_output = node_log_type_val

    config_common_path_val = config_path.perform(context)

    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    cam_pose_array = parse_array_param(cam_pose.perform(context))
    while len(cam_pose_array) < 6:
        cam_pose_array.append('0.0')

    actions.append(LogInfo(msg=f'Using namespace: /{namespace_val}'))
    actions.append(LogInfo(msg=f'Using camera_name: {camera_name_val}'))
    actions.append(LogInfo(msg=f'Using common config: {config_common_path_val}'))
    actions.append(LogInfo(msg=f'Using camera config: {config_camera_path}'))
    actions.append(LogInfo(msg=f'Using object detection config: {object_detection_config_path.perform(context)}'))
    actions.append(LogInfo(msg=f'Using custom object detection config: {custom_object_detection_config_path.perform(context)}'))

    if ros_params_override_path_val:
        actions.append(LogInfo(msg=f'Using ROS override config: {ros_params_override_path_val}'))

    xacro_command = [
        'xacro', ' ',
        xacro_path, ' ',
        'camera_name:=', camera_name_val, ' ',
        'camera_model:=', camera_model_val, ' ',
        'base_frame:=', base_frame, ' ',
        'cam_pos_x:=', cam_pose_array[0], ' ',
        'cam_pos_y:=', cam_pose_array[1], ' ',
        'cam_pos_z:=', cam_pose_array[2], ' ',
        'cam_roll:=', cam_pose_array[3], ' ',
        'cam_pitch:=', cam_pose_array[4], ' ',
        'cam_yaw:=', cam_pose_array[5]
    ]

    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace_val,
        name=f'{camera_name_val}_state_publisher',
        output=node_output,
        parameters=[{
            'use_sim_time': publish_svo_clock,
            'robot_description': Command(xacro_command)
        }],
        remappings=[
            ('robot_description', f'{camera_name_val}_description')
        ]
    )
    actions.append(rsp_node)

    distro = os.environ.get('ROS_DISTRO', 'humble')
    if distro == 'foxy':
        container_exec = 'component_container'
        container_args = ['--ros-args', '--log-level', 'info']
    else:
        container_exec = 'component_container_isolated'
        container_args = ['--use_multi_threaded_executor', '--ros-args', '--log-level', 'info']

    zed_container = ComposableNodeContainer(
        name=container_name_val,
        namespace=namespace_val,
        package='rclcpp_components',
        executable=container_exec,
        arguments=container_args,
        output=node_output,
        composable_node_descriptions=[]
    )
    actions.append(zed_container)

    node_parameters = [
        config_common_path_val,
        config_camera_path,
        object_detection_config_path,
        custom_object_detection_config_path
    ]

    if ros_params_override_path_val:
        node_parameters.append(ros_params_override_path)

    node_parameters.append({
        'use_sim_time': use_sim_time,
        'general.camera_name': camera_name_val,
        'general.camera_model': camera_model_val,
        'svo.svo_path': svo_path,
        'svo.publish_svo_clock': publish_svo_clock,
        'general.serial_number': serial_number,
        'general.camera_id': camera_id,
        'pos_tracking.base_frame': base_frame,
        'pos_tracking.publish_tf': publish_tf,
        'pos_tracking.publish_map_tf': publish_map_tf,
        'sensors.publish_imu_tf': publish_imu_tf
    })

    zed_wrapper_component = ComposableNode(
        package='zed_components',
        plugin='stereolabs::ZedCamera',
        namespace=namespace_val,
        name=node_name_val,
        parameters=node_parameters,
        extra_arguments=[{'use_intra_process_comms': enable_ipc}]
    )

    full_container_name = f'/{namespace_val}/{container_name_val}'
    actions.append(LogInfo(msg=f'Loading ZED node `{node_name_val}` in container `{full_container_name}`'))

    load_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[zed_wrapper_component]
    )
    actions.append(load_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_log_type',
            default_value='screen',
            description='screen | log | both'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace. If empty, camera_name will be used.'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Logical camera name used by ZED parameters.'
        ),
        DeclareLaunchArgument(
            'camera_model',
            description='ZED camera model: zed, zedm, zed2, zed2i, zedx, zedxm, ...'
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='zed_node',
            description='Composable node name.'
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='zed_container',
            description='Composable container name.'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=TextSubstitution(text=default_config_common),
            description='Path to the common YAML config file.'
        ),
        DeclareLaunchArgument(
            'ros_params_override_path',
            default_value='',
            description='Optional extra YAML override file.'
        ),
        DeclareLaunchArgument(
            'object_detection_config_path',
            default_value=TextSubstitution(text=default_object_detection_config_path),
            description='Object detection config YAML.'
        ),
        DeclareLaunchArgument(
            'custom_object_detection_config_path',
            default_value=TextSubstitution(text=default_custom_object_detection_config_path),
            description='Custom object detection config YAML.'
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='0',
            description='Camera serial number.'
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='-1',
            description='Camera ID.'
        ),
        DeclareLaunchArgument(
            'publish_urdf',
            default_value='true',
            description='Start robot_state_publisher.'
        ),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish odom -> camera TF.'
        ),
        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='true',
            description='Publish map -> odom TF.'
        ),
        DeclareLaunchArgument(
            'publish_imu_tf',
            default_value='false',
            description='Publish IMU TF.'
        ),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=TextSubstitution(text=default_xacro_path),
            description='Path to xacro file.'
        ),
        DeclareLaunchArgument(
            'svo_path',
            default_value='live',
            description='SVO input path or live.'
        ),
        DeclareLaunchArgument(
            'publish_svo_clock',
            default_value='false',
            description='Publish SVO clock.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock.'
        ),
        DeclareLaunchArgument(
            'enable_ipc',
            default_value='true',
            description='Enable intra-process comms.'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame name.'
        ),
        DeclareLaunchArgument(
            'cam_pose',
            default_value='[0.0,0.0,0.0,0.0,0.0,0.0]',
            description='Camera pose [x,y,z,roll,pitch,yaw].'
        ),
        OpaqueFunction(function=launch_setup)
    ])