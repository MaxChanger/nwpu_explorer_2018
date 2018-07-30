# 说明

这里包含了两个激光雷达的包和其所需要的依赖包

我们使用的两款激光雷达一个是日本北洋*HOKUYO URG-04LX-UG01* , 另一个是国产思岚*RPLIDAR A2*



## *RPLIDAR A2*

https://github.com/robopeak/rplidar_ros

```bash
sudo　chmod 777 /dev/ttyUSB0             # 或者是别的　根据插入顺序而定
roslaunch rplidar_ros rplidar.launch
```



## *HOKUYO URG-04LX-UG01*

Wiki : http://wiki.ros.org/hokuyo_node

Github : https://github.com/ros-drivers/hokuyo_node

虽然Github上边最新版的驱动是indigo版本的,但是在kinetic下是可以用的

偶然一次发现一个新的驱动

Wiki : http://wiki.ros.org/urg_node/

Github :  https://github.com/ros-drivers/urg_node

```bash
sudo　chmod 777 /dev/ttyACM0             # 或者是别的　根据插入顺序而定
rosrun urg_node urg_node

#或者
rosrun hokuyo_node hokuyo_node
```

