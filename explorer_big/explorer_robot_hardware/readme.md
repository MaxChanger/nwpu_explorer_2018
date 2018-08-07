# explorer_robot_hardware
## _explorer_ 机器人的主要控制中心 _robot_hardware_
这个类中的代码主要用于控制 _explorer_ 机器人,会发送信息到 _explorer_driver_ 中以实现对机器人(下位机)的控制

数据获取依赖于 [_ros_control_](https://github.com/ros-controls/ros_control/tree/indigo-devel "ros_control-indigo 版本的github网址") 的 _indigo_ 版本 

### 潘学谦做出的修改

1. 原版本将一些在不同包内的数据分别写在相应的源代码中间,导致如果需要更改往往需要重新编译,而且容易导致此处的修改没有运用于彼处的毛病,现在将所有参数通过 _lauch_ 文件的命令导入到参数服务器上,并通过 _ros_ 的参数服务器机制导入这些全局数据.目的还是减少编译的次数...
2. 修改 _lauch_ 文件,使得程序使用我自己写的 _explorer_arm_controller_ 具体见包 _explorer_arm_controller_

## _explorer_ 的主程序( _main_ 函数)

由 _controller_manager_ 实现对 

> explorer_arm_controller 
> explorer_vice_wheel_controller
> explorer_joint_state_controller
> explorer_drive_controller

四个 _controller_ 的加载
并由 _controller_manager_ 实现以上类与 _robot_hardware_ 的数据交换

## 当前协议内容

| ID   | 01              | 02            |
| ---- | --------------- | ------------- |
| 1    | 左轮         | 复位 |
| 2    | 右轮     | 右轮发下去 没使用 |
| 3    | 左前副履带速度          | 左后副履带速度        |
| 4    | 右前副履带速度          | 右后副履带速度        |
| 5    | 机械臂底座左右旋转角度       |               |
| 6    | 机械臂整体上下旋转角度 | 小臂上下旋转角度 |
| 7    | 末端执行器轴向旋转   | 末端执行器上下摆动 |
| 8    | 爪子开合        | 爪子轴向旋转 |
| 9    | co2传感器        | 1 |
以上所述的*末端执行器*和*爪子*在当前是指机械爪

注意对机械臂限位的设置

## _ros_control_ 大致介绍

_ros_control_ 通过  _controller_manager_ 将机器人底层控制需要的代码(相应的 _robotHW_ 的子类)与机器人功能实现分离,提高代码的复用性
通过功能实现和实际硬件控制的分离,大大提高了机器人的可扩展性同时,关闭一个功能可以通过直接修改相应的launch文件实现,功能增加和修改不需要重新编译(然并卵...)
很大程度上实现了 实现接口 --> 实现功能 的最方便转换
使用的时候注意最好不要将功能实现写在这个包中间,将物理状况和与底盘的通信数据传送写在这个包中,所用功能实现写在对应(或者新写的)插件中放在 _explorer_controller_ 文件夹中

向底盘发送数据和更改底盘状态最好写在一起
