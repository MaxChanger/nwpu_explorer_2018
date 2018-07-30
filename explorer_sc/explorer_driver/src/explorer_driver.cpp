#include "explorer_driver.h"

RobotDriver::RobotDriver(ros::NodeHandle nh_):
    robot_driver_node__(nh_) {
    robot_serial_data_pub__ = robot_driver_node__.advertise<explorer_msgs::explorer_agreement>("explorer_serial", 10);
    //发送到 pkg explorer_rs484下的节点 node explorer_rs484
    robot_driver_sub__ = robot_driver_node__.subscribe("/explorer_driver", 1000 , &RobotDriver::robotMsgSub, this);
    //接收来自 pkg explorer_hradware 下的节点explorer_hardware
    //robot_driver_sub__ = robot_driver_node__.subscribe("/pub_control",1000 , &RobotDriver::shutdowndriver,this);
}

RobotDriver::~RobotDriver() {

}


//void RobotDriver::shutdowndriver(const std_msgs::String::ConstPtr& ptr)
/**{
  ROS_INFO_STREAM("hello");
    if("shutdowndriver"==ptr->data)
    {
    robot_driver_node__.shutdown();
    }
}*/


/**
 * 将传递过来的float数组(vector)转换为u_int8_t型数组
 * 前方加上 00000100 + id + 11100000 为认证
 */
AgreementMessage RobotDriver::messageToAgreement(int component_id, Message data) {
    explorer_msgs::explorer_agreement serial_message;
    serial_message.msg.resize(3 + data.size() * 4);
    u_int8_t *explorer_message_ptr = serial_message.msg.data();
    explorer_message_ptr[0] = 0x08;    //0100
    explorer_message_ptr[1] = (u_int8_t)component_id;
    explorer_message_ptr[2] = 0xE0;    //1110

    for (int i = 0; i < data.size(); i++) {
        *(float *)(explorer_message_ptr + 3 + 4 * i) = data.at(i);
        //通过强行float,用四位8字节保存;
    }

    ROS_INFO_STREAM("size = " << serial_message.msg.size() << "\n");
    return serial_message;
}

void RobotDriver::robotMsgSub(const explorer_msgs::explorer_messageConstPtr &ptr) {
    if (!ptr->high_level_id) {
        return;
    }

    ros::Rate r(100);
    ROS_INFO_STREAM("id = " << ptr->low_level_id << " size =  " << ptr->data.size() << "\n");
    Message current_data;
    AgreementMessage agreement_msg;
    current_data.resize(ptr->data.size());

    for (int i = 0; i < current_data.size(); i++) {
        current_data.at(i) = ptr->data.at(i);//double 转 float;
    }

    int current_component_id = ptr->low_level_id;
    agreement_msg = messageToAgreement(current_component_id, current_data); //生成agreement
    robot_serial_data_pub__.publish(agreement_msg);
    r.sleep();
}


