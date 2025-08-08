
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


PKG = "novatel_oem7_driver"

def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg )

def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)

def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))

def get_override_params():    
    try:
        oem7_param_override_path = os.environ['NOVATEL_OEM7_DRIVER_PARAM_OVERRIDES_PATH']
    except KeyError: # No overrides specified.
        return {}

    return load_yaml(oem7_param_override_path)

def arg(name, default_value, description):
    return DeclareLaunchArgument(name = name, description = description, default_value = default_value)


def generate_launch_description():

    novatel_namespace_arg = DeclareLaunchArgument(
        'novatel_namespace',
        default_value='lexus3/gps/nova',
        description='Namespace for the Novatel GPS driver')
    novatel_ip_arg = DeclareLaunchArgument(
        'novatel_ip',
        default_value='192.168.10.12',
        description='IP address of the Novatel GPS')
    novatel_port_arg = DeclareLaunchArgument(
        'novatel_port',
        default_value='3002',
        description='Port of the Novatel GPS')
    novatel_if_arg = DeclareLaunchArgument(
        'novatel_if',
        default_value='Oem7ReceiverUdp',
        description='Novatel interface type: Oem7ReceiverTcp or Oem7ReceiverUdp')
    novatel_frame_id_arg = DeclareLaunchArgument(
        'novatel_frame_id',
        default_value='map',
        description='Frame ID for Novatel GPS')
    novatel_child_frame_id_arg = DeclareLaunchArgument(
        'novatel_child_frame_id',
        default_value='lexus3/base_link',
        description='Child frame ID for Novatel GPS')
    novatel_current_pose_x_offset_arg = DeclareLaunchArgument(
        'novatel_current_pose_x_offset',
        default_value='0.0',
        description='X offset for current pose of Novatel GPS \
            map_gyor_0: -697237.0 \
            map_zala_0: -639770.0')
    novatel_current_pose_y_offset_arg = DeclareLaunchArgument(
        'novatel_current_pose_y_offset',
        default_value='0.0',
        description='Y offset for current pose of Novatel GPS\
            map_gyor_0: -5285644.0 \
            map_zala_0: -5195040.0')

    gps_node = Node(
        package=PKG,
        namespace=LaunchConfiguration('novatel_namespace'),
        name='gps_nova_b',
        executable='novatel_oem7_driver_exe',

        parameters=[
            get_params("std_msg_handlers.yaml"    ),
            get_params("std_oem7_raw_msgs.yaml"   ),
            get_params("std_msg_topics.yaml"      ),
            get_params("oem7_supported_imus.yaml" ),
            get_params("std_init_commands.yaml"   ),
            {
                # Refer to README for more information on the configuration parameters
                # Receiver IO Parameters
                'oem7_max_io_errors' : 10,
                'oem7_msg_decoder'   : 'Oem7MessageDecoder',
                'oem7_if'            : LaunchConfiguration('novatel_if'),
                'oem7_ip_addr'       : LaunchConfiguration('novatel_ip'),
                'oem7_port'          : LaunchConfiguration('novatel_port'),

                # Topic Parameters
                'oem7_position_source'      : '', # If not set, then uses INSPVA or BESTPOS based on quality
                'oem7_imu_rate'             : 0,
                'oem7_odometry_zero_origin' : False,
                'oem7_odometry_transform'   : False,

                # Debug/Other Parameters
                'oem7_receiver_log_file'       : '',
                'oem7_decoder_log_file'        : '',
                'oem7_strict_receiver_init'    : True,
                'oem7_publish_unknown_oem7raw' : False,
                'oem7_publish_delay'           : 0.0
            },
            get_override_params() # Must be last to override
        ],
        output='screen',
    )
    nova_oem7_convert_node = Node (
        package='lexus_bringup',
        executable='nova_oem7_to_tf',
        name='nova_oem7_to_tf_node',
        namespace=LaunchConfiguration('novatel_namespace'),
        parameters=[{
            "frame_id":       LaunchConfiguration('novatel_frame_id'),
            "child_frame_id": LaunchConfiguration('novatel_child_frame_id'),
            "pos_x_offset":   LaunchConfiguration('novatel_current_pose_x_offset'),
            "pos_y_offset":   LaunchConfiguration('novatel_current_pose_y_offset')
        }],
        output='screen'
    )

    return LaunchDescription([
        novatel_namespace_arg,
        novatel_ip_arg,
        novatel_port_arg,
        novatel_if_arg,
        novatel_frame_id_arg,
        novatel_child_frame_id_arg,
        novatel_current_pose_x_offset_arg,
        novatel_current_pose_y_offset_arg,

        gps_node,
        nova_oem7_convert_node])




 