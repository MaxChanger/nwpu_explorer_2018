#include "robot_hardware.h"
#include <ros/callback_queue.h>

int main(int argc , char **argv) {
    ros::init(argc , argv , "explorer_hardware");
    ros::NodeHandle hardware_node;
    ExplorerHardware explorer_hardware(hardware_node);

    if (!explorer_hardware.start()) {
        // 测试 hardware 是否成功初始化
        // 事实上可以用其他函数
        ROS_ERROR("can not start the robot");
        return 0;
    }

    ros::NodeHandle robot_node ;
    ros::CallbackQueue manager_call_back_quequ;
    robot_node.setCallbackQueue(&manager_call_back_quequ);
    controller_manager::ControllerManager cm(&explorer_hardware, robot_node);
    // 启动一个新的进程接收和发送数据
    ros::AsyncSpinner spinner(1, &manager_call_back_quequ);
    ros::Rate r(explorer_hardware.getFrep());
    spinner.start();
    ROS_WARN("robot hardware init ok!!");

    while (ros::ok()) {
        explorer_hardware.write(explorer_hardware.getCurrentTime() , explorer_hardware.getPeriod());
        // 调用所有的controller的update
        cm.update(explorer_hardware.getCurrentTime() , explorer_hardware.getPeriod());
        explorer_hardware.read(explorer_hardware.getCurrentTime() , explorer_hardware.getPeriod());
        r.sleep();
    }

    explorer_hardware.stop();
    spinner.stop();
    return 0;
}
