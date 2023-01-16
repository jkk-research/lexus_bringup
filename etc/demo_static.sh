#!/bin/bash

##
source ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/setup.bash

echo "[INFO] Starting gps_duro_reference"
screen -dmS gps_duro_reference bash -c 'ros2 launch lexus_bringup gps_duro_reference.launch.py'
echo "[INFO] Starting foxglove_bridge"
screen -dmS foxglove_bridge bash -c 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
echo "[INFO] Starting foxglove"
foxglove-studio "foxglove://open?ds=rosbridge-websocket&ds.url=ws://localhost:8765"




