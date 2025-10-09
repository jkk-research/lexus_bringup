#!/usr/bin/env bash

echo "dev" | sudo -S nvidia-smi -lgc 300,1500

if screen -list | grep -q "\.aw_sensors_arnl"; then
    echo "WARNING: Screen session 'aw_sensors_arnl' is already running."
else
    screen -mdS aw_sensors_arnl bash -c 'source ~/ros2_ws/install/setup.bash && source ~/autoware/install/setup.bash && ros2 launch autoware_launch lexus3_sensors_autoware.launch.py' 
    # sleep 10 
    
    # screen -mdS usb_cam_arnl bash -c 'source ~/ros2_ws/install/setup.bash && source ~/autoware/install/setup.bash && ros2 launch lexus_bringup usb_web_cam1.launch.py'
fi




 