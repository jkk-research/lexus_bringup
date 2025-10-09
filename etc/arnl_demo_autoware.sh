#!/usr/bin/env bash

if screen -list | grep -q "\.aw_base_arnl"; then
    echo "WARNING: Screen session 'aw_base_arnl' is already running."
else
    echo "dev" | sudo -S nvidia-smi -lgc 300,1500
    screen -mdS aw_base_arnl bash -c 'source ~/autoware/install/setup.bash && ros2 launch autoware_launch autoware.launch.xml launch_sensing:=true launch_perception:=true launch_planning:=true launch_control:=true'
fi

# Wait until the /control/vehicle_cmd_gate node is available
counter=1
while ! ros2 node list 2>/dev/null | grep -q "/control/vehicle_cmd_gate"; do
    echo "  Waiting for /control/vehicle_cmd_gate node to be available... [$counter/20]"
    sleep 1
    counter=$((counter + 1))
    if [ "$counter" -gt 60 ]; then
        echo "ERROR: Timeout waiting for /control/vehicle_cmd_gate"
        exit 1
    fi
done

# Set parameters for the /control/vehicle_cmd_gate node
# ros2 param set /control/vehicle_cmd_gate enable_cmd_limit_filter False
# ros2 param set /control/vehicle_cmd_gate use_emergency_handling False
