## microstrain_inertial_driver
## /dev/ttyACM0
## sudo chmod 777 /dev/ttyACM0

# Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# Please consult your device's documentation for supported features

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_driver'
_DEFAULT_PARAMS_FILE = os.path.join(
    get_package_share_directory(_PACKAGE_NAME),
    'microstrain_inertial_driver_common',
    'config',
    'params.yml'
)
_EMPTY_PARAMS_FILE = os.path.join(
    get_package_share_directory(_PACKAGE_NAME),
    'config',
    'empty.yml'
)

def generate_launch_description():
    # Declare arguments with default values
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='Namespace to use when launching the nodes in this launch file')
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value=_PACKAGE_NAME,
        description='Name to give the Microstrain Inertial Driver node')
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Whether or not to log debug information.')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=_EMPTY_PARAMS_FILE,
        description='Path to file that will load additional parameters')

    # Pass an environment variable to the node to determine if it is in debug or not
    inertial_debug_env = SetEnvironmentVariable(
        'MICROSTRAIN_INERTIAL_DEBUG',
        value=LaunchConfiguration('debug'))

    # ****************************************************************** 
    # Microstrain sensor node 
    # ****************************************************************** 
    microstrain_node = Node(
        package    = _PACKAGE_NAME,
        executable = "microstrain_inertial_driver_node",
        name       = LaunchConfiguration('node_name'),
        namespace  = LaunchConfiguration('namespace'),
        parameters = [
            # Load the default params file manually, since this is a ROS params file, we will need to load the file manually
            yaml.safe_load(open(_DEFAULT_PARAMS_FILE, 'r')),

            # If you want to override any settings in the params.yml file, make a new yaml file, and set the value via the params_file arg
            LaunchConfiguration('params_file'),

            # Supported overrides
            {
                "debug" : LaunchConfiguration('debug')
            },
        ]
    )

    return LaunchDescription([
        namespace_arg,
        node_name_arg,
        debug_arg,
        params_file_arg,
        inertial_debug_env,
        microstrain_node
    ])
  

 
 
