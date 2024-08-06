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
ros2 bag record -s mcap -o $FILE1 --max-cache-size 1048576000 --storage-config-file $DIR1/mcap_writer_options.yaml /lexus3/zed2i/zed_node/left/image_rect_color/compressed /lexus3/vehicle_status
