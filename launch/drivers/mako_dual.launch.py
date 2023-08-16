import yaml

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LifecycleNode
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, TimerAction,
                            RegisterEventHandler, EmitEvent, LogInfo, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, FindExecutable)
from launch.events import matches_action
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO

nodename1 = "/mako_ns/mako1"
nodename2 = "/mako_ns/mako2"
cue_line = "Starting continuous image acquisition ..."

def outstr_action(m, s, r: launch.SomeActionsType):
    if s in m:
        return r

def generate_launch_description():

    avt_vimba_camera_ros2_pkg_dir = get_package_share_directory('avt_vimba_camera')
    
    default_params_file1 = \
        Path(avt_vimba_camera_ros2_pkg_dir) / 'config' / 'mako1.yaml'
#        Path(avt_vimba_camera_ros2_pkg_dir) / 'config' / 'mako1_halfres.yaml'
    params_file1 = LaunchConfiguration('params_file1')
    params_file_arg1 = DeclareLaunchArgument('params_file1',
        default_value = str(default_params_file1),
        description='Name or path to the parameters file to use.')
        
    default_params_file2 = \
        Path(avt_vimba_camera_ros2_pkg_dir) / 'config' / 'mako2.yaml'
#        Path(avt_vimba_camera_ros2_pkg_dir) / 'config' / 'mako2_halfres.yaml'
    params_file2 = LaunchConfiguration('params_file2')
    params_file_arg2 = DeclareLaunchArgument('params_file2',
        default_value = str(default_params_file2),
        description='Name or path to the parameters file to use.')

    default_filter_file = \
        Path(avt_vimba_camera_ros2_pkg_dir) / 'config' / 'param_filter.txt'
    filter_file = LaunchConfiguration('param_filter')
    filter_file_arg = DeclareLaunchArgument('param_filter',
        default_value = str(default_filter_file),
        description='Name or path to the list of read-only parameters to be filtered.')
    
    default_init_file = \
        Path(avt_vimba_camera_ros2_pkg_dir) / 'config' / 'initparam.txt'
    init_file = LaunchConfiguration('init_file')
    init_file_arg = DeclareLaunchArgument('init_file',
        default_value = str(default_init_file),
        description='Name or path to the initial/temporary parameters file to use.')


    ip1_cfg = LaunchConfiguration('ip1')
    ip1_arg = DeclareLaunchArgument('ip1')
    
    ip2_cfg = LaunchConfiguration('ip2')
    ip2_arg = DeclareLaunchArgument('ip2')
    
    cam1_ip = "" #str(ip1_cfg)
    cam2_ip = "" #str(ip2_cfg)

    mako_ns = LaunchConfiguration('mako_ns')
    mako_ns_arg = DeclareLaunchArgument('mako_ns', default_value='lexus3/mako_ns')


    init1 = {}
    init2 = {}
    params1 = {}
    params2 = {}
    filter_read_only = True
    filter_feature_only = True
    list_loaded_params = False
    
    # init load
    print('\n[mako loader] Attempting to open initialization file:\n\t', str(default_init_file))
    with open(str(default_init_file), 'r') as f:
        init1 = yaml.safe_load(f)
        init2 = init1.copy()
        print('\t\t-> successfully loaded an object of', type(init1))
        
    # MAKO1 param load
    print('\n[mako loader] Attempting to open parameter file #1:\n\t', str(default_params_file1))
    with open(str(default_params_file1), 'r') as f:
        raw = yaml.safe_load(f)
        print('\t\t-> successfully loaded an object of', type(raw))
    if not (len(raw)): print('\t\t-> error: Failed to load file! (Or it is completely empty. - Why would you do that though?)')
    elif not isinstance(raw, dict): print('\t\t-> error: data type mismatch! - yaml should be convertible into a python dictionary.')
    else:
        for node, attr in raw.items():
            if not isinstance(attr, dict): print("\t\t-> error: data type/structure mismatch! - The second layer seems to be wrong. " +
                                             "(It should look something like this: {*node_name*: {'ros__parameters': {*value pairs*}}} )")
            for atype, vals in attr.items():
                if (atype=='ros__parameters'):
                    if isinstance(vals, dict): params1 = vals
                    else: print("\t\t-> error: The data can not be interpreted as a python dictionary! WHAT HAVE YOU DONE?!")
                else: print("\t\t-> error: input mismatch! - the key in the second layer of the nested dictionary should be 'ros__parameters'." +
                            " (Like this: {*node_name*: {'ros__parameters': {*value pairs*}}} )")
    print('\n[mako loader] Number of loaded parameters: ', len(params1), "[For filtering options see: launch file]")
    
    # MAKO1 filtering (read-only and non-feature parameters)
    if filter_read_only:
        rof = 0
        print('\n[mako loader] Attempting to open filter config:\n\t', str(default_filter_file))
        with open(str(default_filter_file), 'r') as f:
            print('\t\t-> file opened successfully.')
            for line in f:
                if params1.pop("feature/" + line.strip(), 0): rof += 1
        print("\t\t->", rof, " read-only parameters has been removed.")

    else: print("\t\t-> Read-only parameter filtering is turned off.")
    
    if filter_feature_only:
        toclear = []
        if not cam1_ip: cam1_ip = params1.pop("ip", 0)
        for key, val in params1.items():
            if not "feature/" in key: toclear += [key]
        for key in toclear: del params1[key]
        print("\t\t->", len(toclear), " non-feature parameters has been removed.")
        if cam1_ip:
            init1.update({"ip":cam1_ip})
            params1.update({"ip":cam1_ip})
    else: print("\t\t-> Feature-only parameter filtering is turned off.")
    
    print("\t\t-> Number of parameters to be set: ", len(params1))
    if list_loaded_params:
        print("The parameters:\t\t[disable listing at: launch file]\n")
        for i, j in params1.items(): print(str(i) + ": " + str(j))
        print ("\n---\n")
    else: print("\t\t\t--> Listing of parameters is disabled. [see: launch file]")
    
    
    
    # MAKO2 param load
    print('\n[mako loader] Attempting to open parameter file #2:\n\t', str(default_params_file2))
    with open(str(default_params_file2), 'r') as f:
        raw = yaml.safe_load(f)
        print('\t\t-> successfully loaded an object of', type(raw))
    if not (len(raw)): print('\t\t-> error: Failed to load file! (Or it is completely empty. - Why would you do that though?)')
    elif not isinstance(raw, dict): print('\t\t-> error: data type mismatch! - yaml should be convertible into a python dictionary.')
    else:
        for node, attr in raw.items():
            if not isinstance(attr, dict): print("\t\t-> error: data type/structure mismatch! - The second layer seems to be wrong. " +
                                             "(It should look something like this: {*node_name*: {'ros__parameters': {*value pairs*}}} )")
            for atype, vals in attr.items():
                if (atype=='ros__parameters'):
                    if isinstance(vals, dict): params2 = vals
                    else: print("\t\t-> error: The data can not be interpreted as a python dictionary! WHAT HAVE YOU DONE?!")
                else: print("\t\t-> error: input mismatch! - the key in the second layer of the nested dictionary should be 'ros__parameters'." +
                            " (Like this: {*node_name*: {'ros__parameters': {*value pairs*}}} )")
    print('\n[mako loader] Number of loaded parameters: ', len(params2), "[For filtering options see: launch file]")
    
    # MAKO2 filtering (read-only and non-feature parameters)
    if filter_read_only:
        rof = 0
        print('\n[mako loader] Attempting to open filter config:\n\t', str(default_filter_file))
        with open(str(default_filter_file), 'r') as f:
            print('\t\t-> file opened successfully.')
            for line in f:
                if params2.pop("feature/" + line.strip(), 0): rof += 1
        print("\t\t->", rof, " read-only parameters has been removed.")

    else: print("\t\t-> Read-only parameter filtering is turned off.")
    
    if filter_feature_only:
        toclear = []
        if not cam2_ip: cam2_ip = params2.pop("ip", 0)
        for key, val in params2.items():
            if not "feature/" in key: toclear += [key]
        for key in toclear: del params2[key]
        print("\t\t->", len(toclear), " non-feature parameters has been removed.")
        if cam2_ip:
            init2.update({"ip":cam2_ip})
            params2.update({"ip":cam2_ip})
    else: print("\t\t-> Feature-only parameter filtering is turned off.")
    
    print("\t\t-> Number of parameters to be set: ", len(params2))
    if list_loaded_params:
        print("The parameters:\t\t[disable listing at: launch file]\n")
        for i, j in params2.items(): print(str(i) + ": " + str(j))
        print ("\n---\n")
    else: print("\t\t\t--> Listing of parameters is disabled. [see: launch file]")
    
    
    
    if True: #not cam1_ip:
#        cam1_ip=input("Missing IP address for mako camera #1, please type it in: ")
        init1.update({"ip":cam1_ip})
        params1.update({"ip":cam1_ip})
        
    if True: #not cam2_ip:
#        cam2_ip=input("Missing IP address for mako camera #2, please type it in: ")
        init2.update({"ip":cam2_ip})
        params2.update({"ip":cam2_ip})
            
    print('\n[mako loader] Launching node...')
    
    # MAKO1
    # temporary camera node (1) with fail-safe parameters (2-step parameter loading procedure for camera #1)
    mako1_init = Node(
            package='avt_vimba_camera',
            namespace=nodename1.rsplit('/',1)[0],
            executable='mono_camera_exec',
            name='mako1_init_temp',
            parameters = [init1],
            )
    
    # MAKO1
    # actual active mako1 camera node
    mako1 = Node(
            package='avt_vimba_camera',
            namespace=nodename1.rsplit('/',1)[0],
            executable='mono_camera_exec',
            name=nodename1.rsplit('/',1)[1],
            parameters = [params1],
            )
            
    # MAKO2
    # temporary camera node (2) with fail-safe parameters (2-step parameter loading procedure for camera #2)
    mako2_init = Node(
            package='avt_vimba_camera',
            namespace=nodename2.rsplit('/',1)[0],
            executable='mono_camera_exec',
            name='mako2_init_temp',
            parameters = [init2],
            )
    
    # MAKO2
    # actual active mako2 camera node
    mako2 = Node(
            package='avt_vimba_camera',
            namespace=nodename2.rsplit('/',1)[0],
            executable='mono_camera_exec',
            name=nodename2.rsplit('/',1)[1],
            parameters = [params2],
            )
    
    # kills temporary node (1) and starts mako2 launching procedure
    relaunch1 = ExecuteProcess(
            cmd=[["pkill -2 -u $USER mono_camera_exe"]],
            shell=True,
            on_exit=mako2_init
        )
        
    # kills temporary node (2) and then starts mako1 and mako2 with the actual parameters (ensures proper parameter loading)
    relaunch2 = ExecuteProcess(
            cmd=[["pkill -2 -u $USER mono_camera_exe"]],
            shell=True,
            on_exit=mako1
        )

    # triggers when temporary node (1) is ready, starting the next step
    io_trig1 = RegisterEventHandler(
        OnProcessIO(
            target_action=mako1_init,
            on_stdout=lambda info: outstr_action(str(info.text), cue_line, relaunch1),
            on_stderr=lambda info: outstr_action(str(info.text), cue_line, relaunch1)
        ),
#        handle_once=True
    )

    # triggers when temporary node (2) is ready, starting the next step
    io_trig2 = RegisterEventHandler(
        OnProcessIO(
            target_action=mako2_init,
            on_stdout=lambda info: outstr_action(str(info.text), cue_line, relaunch2),
            on_stderr=lambda info: outstr_action(str(info.text), cue_line, relaunch2)
        ),
#        handle_once=True
    )

    # triggers when first node (1) is ready, starting the next node
    io_trig3 = RegisterEventHandler(
        OnProcessIO(
            target_action=mako1,
            on_stdout=lambda info: outstr_action(str(info.text), cue_line, mako2),
            on_stderr=lambda info: outstr_action(str(info.text), cue_line, mako2)
        ),
#        handle_once=True
    )
        
    ld = launch.LaunchDescription([
#        ip1_arg,
#        ip2_arg,
        mako1_init,
        io_trig1,
        io_trig2,
        io_trig3
    ])

    return ld

