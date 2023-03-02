#!/bin/bash

##
source ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/setup.bash

echo "[INFO] Starting foxglove_bridge"
screen -dmS foxglove_bridge bash -c 'ros2 launch lexus_bringup foxglove_bridge_launch.xml'
echo "[INFO] Starting gps_duro_reference"
screen -dmS gps_duro_reference bash -c 'ros2 launch lexus_bringup gps_duro_reference.launch.py'
echo "[INFO] Starting os_64_center_a"
screen -dmS os_64_center_a bash -c 'ros2 launch lexus_bringup os_64_center_a.launch.py'
echo "[INFO] Starting os_32_right_a"
screen -dmS os_32_right_a bash -c 'ros2 launch lexus_bringup os_32_right_a.launch.py'
echo "[INFO] Starting tf_static"
screen -dmS tf_static bash -c 'ros2 launch lexus_bringup tf_static.launch.py'
echo "[INFO] Starting 3d_marker"
screen -dmS 3d_marker bash -c 'ros2 launch lexus_bringup 3d_marker.launch.py'
echo "[INFO] Starting zed_default_a.launch.py"
screen -dmS zed_default_a.launch.py bash -c 'ros2 launch lexus_bringup zed_default_a.launch.py'
echo "[INFO] Starting rviz00.launch.py"
screen -dmS rviz00.launch.py bash -c 'ros2 launch lexus_bringup rviz00.launch.py'
echo "[INFO] Starting can_pacmod3.launch.xml"
screen -dmS can_pacmod3 bash -c 'ros2 launch lexus_bringup can_pacmod3.launch.xml'
#echo "[INFO] Starting foxglove"
#foxglove-studio "foxglove://open?ds=rosbridge-websocket&ds.url=ws://localhost:8765"




