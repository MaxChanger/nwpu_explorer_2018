![1](/home/sun/WorkSpace/Ros_workspace/catkin_ws/src/nwpu_explorer/image/1.png)

# NWPU_Explorer

NWPU Explorer Code Repository — 西北工业大学舞蹈机器人基地救援组

- 软件组：黄德慧、孙家岱、赵玉凡、刘靖泽
- 电子组：梁成栋、谢凯、~~丁晓斌~~
- 机械组：闫雨晨

 

## explorer_small 编译时的依赖包说明目录

***

- **controller_interface**     robothardware的核心控件
- **moveit_msg**                    moveit 消息依赖
- **control_toolbox**             control依赖
- **controller_manager**      controllers 管理
- **joint_limits_interface**    机械臂等节点的接口
- **transmission_interface** 机械臂moveit需要

```bash
sudo apt install ros-kinetic-moveit ros-kinetic-joy ros-kinetic-controller-interface  ros-kinetic-joint-limits-interface  ros-kinetic-transmission-interface ros-kinetic-realtime-tools ros-kinetic-control-toolbox ros-kinetic-controller-manager 
```

## explorer_navigation 编译时的依赖包说明目录

```bash
sudo apt install ros-kinetic-costmap-2d
sudo apt install ros-kinetic-nav-core
```

## explorer_vision编译注意事项
#### explorer_qrcode编译依赖
- **zbar**

```bash
wget http://downloads.sourceforge.net/project/zbar/zbar/0.10/zbar-0.10.tar.gz
tar -zvxf zbar-0.10.tar.gz
sudo apt-get install python-gtk2-dev
sudo apt-get install libqt4-dev
export CFLAGS=""
./configure --without-imagemagick --disable-video 
make
sudo make install
```
## explorer_vision

此文件夹中为启动摄像头的launch文件，使用前确认安装usb_cam

```bash
sudo apt install ros-kinetic-usb-cam
```



## flir_one_node

使用前先将package中的51-usb-flir-one.rules复制到/etc/udev/rules.d/中即可识别usb转接头接入的flir one热成像仪。

rules手记：http://www.cnblogs.com/sopost/archive/2013/01/09/2853200.html



## explorer_pyaudio

pyaudio的机制为在本地和工控机上分别运行talker和listener四个节点，通过socket转发声音数据实现。使用方法为分别在本地和工控机上的launch文件中，将对应的local地址改为操作者电脑的ip地址再行启动。如果接入外部麦克风，需要在设置中特别指定声音输入和输出硬件（可通过远程桌面实现）。



## remmina远程桌面配置

www.cnblogs.com/xuliangxing/p/7642650.html



## usb_cam

视觉通过usb_cam可以灵活实现。通过更改usb_cam包中的对应的五个video0,1,2,3,4并指定参数可以灵活启动不同摄像头。需要注意对于usb2.0协议摄像头，一个工控机上3.0usb口即使通过usb分线器也最多只能接入两个320*240分辨率，每秒30帧的摄像头。

