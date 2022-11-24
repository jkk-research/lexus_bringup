# `lexus_bringup` ROS2 package
🚗 ROS2 package for basic functions on Lexus rx450h



```
cd ~/ros2_ws/src
git clone https://github.com/jkk-research/lexus_bringup
colcon build --packages-select lexus_bringup
```

```
source ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch lexus_bringup drivers/can_pacmod3.launch.xml
```



![](https://raw.githubusercontent.com/jkk-research/lexus_base/main/img/lexus3d01.gif)