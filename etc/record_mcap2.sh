#!/bin/bash

## ROS 2 rosbag recorder (MCAP format)

cd "$(dirname "$0")"
DIR1="$(pwd)"
cd /mnt/bag
mkdir -p $(date -I)
cd $(date -I)
TEXT1="$1"
TIME1="$(date +"%Y-%m-%d_%H-%M")"
FILE1="$TEXT1$TIME1"
FILE2="x_rosparam_dump_$TEXT1$TIME1.txt"
PWD1="$(pwd)"
echo "Writing to file: $PWD1/$FILE1"
ros2 bag record -s mcap -o $FILE1 --max-cache-size 1048576000 --storage-config-file $DIR1/mcap_writer_options.yaml /tf /tf_static /lexus3/gps/duro/current_pose /lexus3/gps/duro/current_pose_fake_orientation /lexus3/gps/duro/current_pose_with_cov /lexus3/gps/duro/imu /lexus3/gps/duro/mag /lexus3/gps/duro/navsatfix /lexus3/gps/duro/odom /lexus3/gps/duro/rollpitchyaw /lexus3/gps/duro/rollpitchyaw_fake /lexus3/gps/duro/status_flag /lexus3/gps/duro/status_string /lexus3/gps/duro/time_diff /lexus3/gps/duro/time_ref /lexus3/os_center/imu /lexus3/os_center/points /lexus3/os_left/points /lexus3/os_right/points
