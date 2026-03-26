
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "novatel_oem7_driver"
lx_nova_namespace = '/lexus3/gps/nova'


def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory(PKG), "config", cfg)


def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)


def get_params(cfg):
    return load_yaml(get_cfg_path(cfg))


def get_override_params():
    try:
        oem7_param_override_path = os.environ['NOVATEL_OEM7_DRIVER_PARAM_OVERRIDES_PATH']
    except KeyError:  # No overrides specified.
        return {}

    return load_yaml(oem7_param_override_path)


def arg(name, default_value, description):
    return DeclareLaunchArgument(name=name, description=description, default_value=default_value)


def generate_launch_description():

    nova_node = Node(
        package=PKG,
        namespace=lx_nova_namespace,
        name='main',
        executable='novatel_oem7_driver_exe',

        parameters=[
             get_params("std_msg_handlers.yaml"),
             get_params("std_oem7_raw_msgs.yaml"),
             get_params("std_msg_topics.yaml"),
             get_params("oem7_supported_imus.yaml"),
             get_params("std_init_commands.yaml"),
             {
                 # Refer to README for more information on the configuration parameters
                 # Receiver IO Parameters
                 'oem7_max_io_errors': 10,
                 'oem7_msg_decoder': 'Oem7MessageDecoder',
                 'oem7_if': LaunchConfiguration('oem7_if'),
                 'oem7_ip_addr': LaunchConfiguration('oem7_ip_addr'),
                 'oem7_port': LaunchConfiguration('oem7_port'),

                 # Topic Parameters
                 # If not set, then uses INSPVA or BESTPOS based on quality
                 'oem7_position_source': '',
                 'oem7_imu_rate': 0,
                 'oem7_odometry_zero_origin': False,
                 'oem7_odometry_transform': False,

                 # Debug/Other Parameters
                 'oem7_receiver_log_file': '',
                 'oem7_decoder_log_file': '',
                 'oem7_strict_receiver_init': True,
                 'oem7_publish_unknown_oem7raw': False,
                 'oem7_publish_delay': 0.0

             },
            get_override_params()  # Must be last to override
        ],

        output='screen',
    )

    convert_node = Node(
        package="lexus_bringup",
        namespace=lx_nova_namespace,
        name='gps_convert',
        executable='nova_oem7_to_tf',
        parameters=[
             {
                'use_sim_time': False,
                'x_coord_offset': -697237.0,  # map_gyor_0
                # 'x_coord_offset': -639770.0, # map_zala_0
                'y_coord_offset': -5285644.0,  # map_gyor_0
                # 'y_coord_offset': -5195040.0, # map_zala_0
                'z_coord_exact_height': 1.9,
                'z_coord_offset_plus': -112.0,
                'frame_id': 'map',
                'child_frame_id': 'lexus3/base_link',
                # z_coord_ref_switch can be exact / zero_based / orig / orig_offset
                # exact: the Z coorindinate is always z_coord_exact_height param (must be set in this launch)
                # zero_based: Z coordinate starts from 0 and relative
                # orig: the original Z provided by sensor
                # orig_offset: the original Z plus a fixed offset
                'z_coord_ref_switch': 'orig_offset',
                'pose_topic_pub': 'current_pose',
                'utm_topic_sub': 'bestutm',
                'inspva_topic_sub': 'inspva',
                'debug_publish': False,
             }
        ]
    )

    ip_arg = arg('oem7_ip_addr', '192.168.10.12',
                 'IP Address of Oem7 Receiver, e.g. 192.168.1.2')
    port_arg = arg('oem7_port',   '3001',
                   'TCP or UDP port, e.g. 3002')
    if_arg = arg('oem7_if',     'Oem7ReceiverUdp',
                 'Interface Type: Oem7ReceiverTcp or Oem7ReceiverUdp')

    return LaunchDescription([ip_arg, port_arg, if_arg,
                              nova_node, convert_node])
