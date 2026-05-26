# `2026.L` PacMOD ROS2 

Communtations with vehicle happens through */pacmod* topics, using the vehicles CAN bus network.


Within the Spectra PC we have a Kvaser PCIEcan 4xHS card that supports native communication through the can bus. Currently we' using the default Linux SocketCAN to acess the device, and the connected CAN-bus, on the can0 channel. 

To start the ROS2 driver:

```bash
ros2 launch lexus_bringup can_and_status.launch.py
```






 