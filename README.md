# `lexus_bringup` ROS2 package
🚗 ROS2 package for basic functions on Lexus rx450h

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

## Build

It is assumed that the workspace is `~/ros2_ws/`.

### `Terminal 1` 🔴 clone

```
cd ~/ros2_ws/src
git clone https://github.com/jkk-research/lexus_bringup
```

### `Terminal 1` 🔴 build
```
cd ~/ros2_ws
colcon build --packages-select lexus_bringup
```

### `Terminal 2` 🔵 run
```
source ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch lexus_bringup gps_duro_reference.launch.py
ros2 launch lexus_bringup can_pacmod3.launch.xml
ros2 launch lexus_bringup os_64_center_a.launch.py 
ros2 launch lexus_bringup zed_default_a.launch.py 
ros2 launch lexus_bringup rviz00.launch.py 
ros2 run    lexus_bringup speed_control
```

## Useful
``` bash
dev@u22glx:~$ colcon_cd lexus_bringup
dev@u22glx:~/ros2_ws/src/lexus_bringup(main)$ 
```

``` c
ros2 interface show pacmod3_msgs/msg/SystemCmdFloat
```


``` c
ros2 run tf2_ros static_transform_publisher --x 697237.0 --y 5285644.0 --z 0.0 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 --frame-id map --child-frame-id map_gyor_0
ros2 run tf2_ros static_transform_publisher --x 639770.0 --y 5195040.0 --z 0.0 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 --frame-id map --child-frame-id map_zala_0
```

``` yaml
ros2 topic pub /pacmod/parsed_tx/vehicle_speed_rpt pacmod3_msgs/msg/VehicleSpeedRpt "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, vehicle_speed: 0.1, vehicle_speed_valid: true}"
```


![](https://raw.githubusercontent.com/jkk-research/lexus_base/main/img/lexus3d01.gif)


## Zed install
```
colcon build --symlink-install --packages-select zed_interfaces zed_components zed_wrapper zed_ros2 --cmake-args=-DCMAKE_BUILD_TYPE=Release
```
## Duro install
```
cd ~/ros2_ws/src
git clone https://github.com/szenergy/duro_gps_driver/
cd ~/ros2_ws/src/duro_gps_driver
git checkout ros2-humble
cd ~/ros2_ws
colcon build --packages-select duro_gps_driver
```

## 3D marker install
```
cd ~/ros2_ws/src
git clone https://github.com/szenergy/rviz_markers
cd ~/ros2_ws/src/rviz_markers
git checkout ros2-humble
cd ~/ros2_ws
colcon build --packages-select rviz_markers
```
## GUI install 
```
cd ~/ros2_ws/src
git clone https://github.com/jkk-research/gui_lexus
cd ~/ros2_ws
colcon build --packages-select gui_lexus
```
## CAN - PACMod3 - Kvaser install

- [Install Pacmod3](https://github.com/astuff/pacmod3#installation)
- [Install Kvaser](https://www.kvaser.com/canlib-webhelp/section_install_linux.htm)
- [Install Kvaser ROS Interface](https://github.com/astuff/kvaser_interface#installation)

## Mako install 
```
cd ~/ros2_ws/src
git clone https://github.com/astuff/avt_vimba_camera.git
cd ~/ros2_ws/src/avt_vimba_camera
git checkout ros2_master
cd ~/ros2_ws
colcon build --packages-select avt_vimba_camera
```
