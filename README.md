### explorer_sc 编译时的依赖包说明目录

***

- **controller_interface**     robothardware的核心控件
- **moveit_msg**                    moveit 消息依赖
- **control_toolbox**             control依赖
- **controller_manager**      controllers 管理
- **joint_limits_interface**    机械臂等节点的接口
- **transmission_interface** 机械臂moveit需要

```bash

sudo apt install ros-kinetic-moveit ros-kinetic-controller-interface  ros-kinetic-joint-limits-interface  ros-kinetic-transmission-interface ros-kinetic-realtime-tools ros-kinetic-control-toolbox ros-kinetic-controller-manager 
```

## explorer_navigation 编译时的依赖包说明目录

```bash
sudo apt install ros-kinetic-costmap-2d
sudo apt install ros-kinetic-nav-core
```

