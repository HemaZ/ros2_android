# ros2_android
ROS2 example packages to use your android device sensors and camera.

Download this apps on your Android device [CamON Live Streaming‏](https://play.google.com/store/apps/details?id=com.spynet.camon&hl=ar).

Then You can run the ros node as follow

`ros2 run android_node android --ros-args -p "ip:=DEVICE_IP" -p port:=PORT -p fps:=15 -p gps_freq:=2`

replace **DEVICE_IP** and **PORT** with Your Device IP and Port number from the app.

### optional parameters:

* fps : camera topic rate Default:15
* gps_freq : GPS topic rate Default:2

### Available services: 
| Service name  | Type                             | Request     |
|---------------|----------------------------------|-------------|
| /camera_zoom  | android_node_interfaces.srv.Zoom | [-100,100]  |
| /toggle_torch | android_node_interfaces.srv.Torc | True, False |

### Build status:

![Build](https://github.com/HemaZ/ros2_android/workflows/Build/badge.svg)