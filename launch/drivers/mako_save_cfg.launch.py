import yaml
import datetime
import readline
from os.path import isfile

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, ExecuteProcess)
from launch.substitutions import (LaunchConfiguration, FindExecutable)

def din(cfg, defval=''):
   readline.set_startup_hook(lambda: readline.insert_text(defval))
   try:
      return input(cfg)
   finally:
      readline.set_startup_hook()
      
cmd_info_prefix = "[mako_cfg_save]: "

def checkfile(f):
    if isfile(f):
        return (cmd_info_prefix + "Configuration file successfully saved at: " + f + '!')
    else:
        return (cmd_info_prefix + "Error: cannot see file. Please check if it has been saved to: "+ f + '!' +
                  "\n - (Default save action is to append file to prevent accidental overwriting.)")

def generate_launch_description():

    print(cmd_info_prefix + "Preparing to save the configuration of a (singular) Allied Vision Technologies Mako camera from a ros2 node...")

#    filedir = input(" - Please type in a custom save directory or press Enter to automatically find and select the lexus_bringup package default 'mako_cfg' directory!\n >")
    filedir = input(" - save directory: ")
    if not filedir:
        lexus_bringup_pkg_dir = get_package_share_directory('lexus_bringup')
        cfg_dir = Path(lexus_bringup_pkg_dir) / 'launch' / 'drivers' / 'mako_cfg'

    if filedir:
#        print(filedir)
        if filedir[-1] != '/': filedir = filedir + '/'
    else: filedir = input(" - Error: Directory not found. Please type the path in manually!\n >")

    nodename = din(" - node name: ", "/lexus3/mako1")

    file_iter = 1
    datestamp = datetime.date.today().strftime("%Y-%m-%d")
    base_name = nodename.replace("/", "_") + "_" + datestamp
    if base_name[0] == '_': base_name = base_name[1:]
    filename = base_name + ".yaml"
    while isfile(filedir + filename):
        file_iter += 1
        filename = base_name + '_' + str(file_iter) + ".yaml"

    filename = din(" - filename: ", filename)
        
    report = LogInfo(msg = checkfile(filedir+filename))

    save_it = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " param dump ",
            nodename,
            " >> ",
            filedir,
            filename
        ]],
        shell=True,
        on_exit = report
        )
        
    ld = launch.LaunchDescription([
    save_it
    ])

    return ld
