#!/bin/bash

##

echo "[INFO] Starting foxglove_bridge"
screen -dmS foxglove_bridge bash -c 'ros2 launch lexus_bringup foxglove_bridge_launch.xml'
echo "[INFO] Starting foxglove"
foxglove-studio "foxglove://open?ds=rosbridge-websocket&ds.url=ws://localhost:8765"
