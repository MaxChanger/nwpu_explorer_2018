#include "explorer_thread.h"


explorer_thread::explorer_thread(QObject *parent):
    QThread (parent)
{
    stopped = false;
}
void explorer_thread::run(){

    //spin()运行ros 并不断的接受消息!!!!!!
    while(!stopped){
    //ROS_INFO("WWWWWWWWWWWTTTTTTTTTTFFFFFFFFFFF");
    ros::Subscriber topic_vice_wheel_sub;
    ros::Subscriber topic_wheel_sub;
    ros::Subscriber topic_arm_sub;
    ros::Subscriber topic_co2_sub;
    //geometry_msgs::Twist wheel_msg;
    //explorer_msgs::explorer_vice_wheel vice_whee_msg;
    int argc = 0; char **argv = NULL;
    ros::init(argc, argv, "client_plug");
    if (!ros::master::check())
    {
        ROS_INFO("No master started!");
        //this->close();
    }
    ros::start(); 
    // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    topic_wheel_sub = n.subscribe<geometry_msgs::Twist>("/explorer_drive_controller/cmd_vel",2,&explorer_thread::topic_wheel_change,this);
    topic_vice_wheel_sub = n.subscribe<explorer_msgs::explorer_vice_wheel>("/explorer_vice_wheel_controller/explorer_vice_wheel",2 ,&explorer_thread::topic_vice_wheel_change,this);
    topic_arm_sub = n.subscribe<sensor_msgs::JointState>("/joint_states",2,&explorer_thread::topic_arm_change ,this);
    topic_co2_sub = n.subscribe<explorer_msgs::explorer_co2>("explorer_serial_data/19",2,&explorer_thread::topic_co2_change,this);
    ros::spin();
}
    stopped = false;
}


void explorer_thread::stop(){
    stopped = true;
}


//将数据接受下来并进行发射Qt信号
void explorer_thread::topic_wheel_change(geometry_msgs::Twist ptr){
        Q_EMIT signal_wheel(ptr);
}
void explorer_thread::topic_vice_wheel_change(explorer_msgs::explorer_vice_wheel ptr){
        Q_EMIT signal_vice_wheel(ptr);
}
void explorer_thread::topic_arm_change(sensor_msgs::JointState ptr){
    if(ptr.name[0] == "arm1_bearing_joint"){
        Q_EMIT signal_arm(ptr);
    }
}
void explorer_thread::topic_co2_change(explorer_msgs::explorer_co2 ptr){
    Q_EMIT signal_co2(ptr);
}
