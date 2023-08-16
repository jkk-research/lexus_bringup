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

nodename = "/lexus3/mako2"
cue_line = "Starting continuous image acquisition ..."

def outstr_action(m, s, r: launch.SomeActionsType):
    if s in m:
        return r

def generate_launch_description():

    lexus_bringup_pkg_dir = get_package_share_directory('lexus_bringup')
    cfg_dir = Path(lexus_bringup_pkg_dir) / 'launch' / 'drivers' / 'mako_cfg'
    
    default_params_file = cfg_dir / 'mako2.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
        default_value = str(default_params_file),
        description='Name or path to the parameters file to use.')

    default_filter_file = cfg_dir / 'param_filter.txt'
    params_file = LaunchConfiguration('param_filter')
    params_file_arg = DeclareLaunchArgument('param_filter',
        default_value = str(default_filter_file),
        description='Name or path to the list of read-only parameters to be filtered.')
    
    default_init_file = cfg_dir / 'initparam.txt'
    init_file = LaunchConfiguration('init_file')
    init_file_arg = DeclareLaunchArgument('init_file',
        default_value = str(default_init_file),
        description='Name or path to the initial/temporary parameters file to use.')


    ip2_cfg = LaunchConfiguration('ip2')
    ip2_arg = DeclareLaunchArgument('ip2')
    
    cam2_ip = "" #str(ip2_cfg)

    mako_ns = LaunchConfiguration('mako_ns')
    mako_ns_arg = DeclareLaunchArgument('mako_ns', default_value='lexus3')

    params = {}
    init = {}
    filter_read_only = True
    filter_feature_only = True
    list_loaded_params = False
    
    #init load
    print('\n[mako loader] Attempting to open initialization file:\n\t', str(default_init_file))
    with open(str(default_init_file), 'r') as f:
        init = yaml.safe_load(f)
        print('\t\t-> successfully loaded an object of', type(init))
        
    #param load
    print('\n[mako loader] Attempting to open parameter file:\n\t', str(default_params_file))
    with open(str(default_params_file), 'r') as f:
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
                    if isinstance(vals, dict): params = vals
                    else: print("\t\t-> error: The data can not be interpreted as a python dictionary! WHAT HAVE YOU DONE?!")
                else: print("\t\t-> error: input mismatch! - the key in the second layer of the nested dictionary should be 'ros__parameters'." +
                            " (Like this: {*node_name*: {'ros__parameters': {*value pairs*}}} )")
    print('\n[mako loader] Number of loaded parameters: ', len(params), "[For filtering options see: launch file]")
    
    #filtering (read-only and non-feature parameters)
    if filter_read_only:
        rof = 0
        print('\n[mako loader] Attempting to open filter config:\n\t', str(default_filter_file))
        with open(str(default_filter_file), 'r') as f:
            print('\t\t-> file opened successfully.')
            for line in f:
                if params.pop("feature/" + line.strip(), 0): rof += 1
        print("\t\t->", rof, " read-only parameters has been removed.")

    else: print("\t\t-> Read-only parameter filtering is turned off.")
    
    if filter_feature_only:
        toclear = []
        if not cam2_ip: cam2_ip = params.pop("ip", 0)
        for key, val in params.items():
            if not "feature/" in key: toclear += [key]
        for key in toclear: del params[key]
        print("\t\t->", len(toclear), " non-feature parameters has been removed.")
        if cam2_ip:
            init.update({"ip":cam2_ip})
            params.update({"ip":cam2_ip})
    else: print("\t\t-> Feature-only parameter filtering is turned off.")
    
    print("\t\t-> Number of parameters to be set: ", len(params))
    if list_loaded_params:
        print("The parameters:\t\t[disable listing at: launch file]\n")
        for i, j in params.items(): print(str(i) + ": " + str(j))
        print ("\n---\n")
    else: print("\t\t\t--> Listing of parameters is disabled. [see: launch file]")
    
    if True: #not cam2_ip:
#        cam2_ip=input("Missing IP address for mako camera #2, please type it in: ")
        init.update({"ip":cam2_ip})
        params.update({"ip":cam2_ip})
            
    print('\n[mako loader] Launching node...')
    
    #temporary camera node with fail-safe parameters (2-step parameter loading procedure)
    mako1_init = Node(
            package='avt_vimba_camera',
            namespace=nodename.rsplit('/',1)[0],
            executable='mono_camera_exec',
            name='mako1_init_temp',
            parameters = [init],
            )
    
    #actual active camera node
    mako1 = Node(
            package='avt_vimba_camera',
            namespace=nodename.rsplit('/',1)[0],
            executable='mono_camera_exec',
            name=nodename.rsplit('/',1)[1],
            parameters = [params],
            )
    
    #restarts mako1 with the actual parameters (ensures proper parameter loading)
    relaunch = ExecuteProcess(
            cmd=[["pkill -2 -u $USER mono_camera_exe"]],
            shell=True,
            on_exit=mako1
        )

    io_trig = RegisterEventHandler(
        OnProcessIO(
            target_action=mako1_init,
            on_stdout=lambda info: outstr_action(str(info.text), cue_line, relaunch),
            on_stderr=lambda info: outstr_action(str(info.text), cue_line, relaunch)
        ),
#        handle_once=True
    )
        
    ld = launch.LaunchDescription([
#        ip1_arg,
        mako1_init,
        io_trig
    ])

    return ld

