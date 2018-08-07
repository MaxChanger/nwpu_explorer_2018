# *explorer_arm_controller*

## 概述

这个包主要用于 *explorer* 机器人的机械臂控制

### 控制概述

目前版本的控制仅仅是通过向将机械臂的目标位置逼近的方法实现.

注意
1. 每次到来的移动指令都会从当前位置开始改变当前机械臂的目标.
2. 在收到数据时有时需要判断数据实时性

### 控制机理

这个控制器仅仅完成位置接受和速度控制两个功能，具体通过封装在 _joint_controller.h_ 文件中

如果出现当前无法实现的关节，可以重载上述文件中的 _joint_ 类实现

### 对外接口

| 接口名称                | 消息类型                                | 用途                 |
| ------------------- | ----------------------------------- | ------------------ |
| explorer_arm_driect | geometry_msgs::TwistStamped         | 用于跳过解算直接控制机械臂移动的情况 |
| explorer_reset      | explorer_msgs::explorer_reset       | 用于复原机械臂的位置         |
| explorer_paw        | std_msgs::Float32                   | 用于控制机械爪的开合         |
| explorer_moveit     | explorer_msgs::explorer_armConstPtr | 用于接受解算的结果          |

## 控制数据来源

来自于参数服务器 */explorer_arm_controller* 的数据

参数设置于 *$(find explorer_robot_hardware)/config/explorer_arm_controller.yaml* 

编写者 西工大舞蹈机器人基地 救援组 软件组 潘学谦

## 展望

当前版本机械臂没有受力反馈,如果希望机械臂更近一步,需要增加各个关节的力反馈

机械臂 $\to$ 能够主动抓取的机械臂 $\to$ 具有示教功能的机械臂 $\to$ 主动移动的机械脚 