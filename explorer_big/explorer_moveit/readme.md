### MovieIt的坐标发送

根据手柄的消息 来判断出机械臂末端执行器(也就是爪子)的位置和姿态的信息,位置用坐标系 姿态用rpy 之后传入的时候改成四元数 确定好坐标之后,将坐标通过setPostTarget来将其发布出去
之后通过listener包进行规划结果的监听和下发

参考:
https://www.cnblogs.com/zxouxuewei/p/6092759.html 周学伟的博客 整体上讲解了MoveIt的结构
http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html API接口详解!!!!!!(注意使用和参考的版本问题)

实践参考博客:
https://blog.csdn.net/q71018188/article/details/78977358 官方案例(中文+部分讲解)
https://blog.csdn.net/qq_38288618/article/details/79039616 三篇博客 较为完整
https://blog.csdn.net/xu1129005165/article/details/70037698 简单的实现 

