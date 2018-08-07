#ifndef EXPLORER_JOY_CONTROL_H
#define EXPLORER_JOY_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <explorer_msgs/explorer_reset.h>
#include <explorer_msgs/explorer_vice_wheel.h>
#include <explorer_msgs/explorer_arm.h>
#include <explorer_msgs/explorer_vice_reset.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "explorer_joy_explainer.hpp"
#include <sstream>
#include <explorer_msgs/explorer_moveit_paw.h>
//#include <opencv2/highgui/highgui.hpp>    
//#include "opencv2/core/version.hpp"
template <typename T>
bool operator == (const T &a, const T &b) {
    std::ostringstream as, bs;
    ros::message_operations::Printer<T>::stream(as, "", a);
    ros::message_operations::Printer<T>::stream(bs, "", b);
    return as.str() == bs.str();
}
template <typename T>
bool operator != (const T &a, const T &b) {
    return !(a == b);
}
class ExplorerTeleop {
public:
    ExplorerTeleop();
    ~ExplorerTeleop();
protected:
    void messageClean();

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void publishControl();
    void vice_wheel_speed_change(std_msgs::Float32);
    void front_speed_change(std_msgs::Float32);
    void rotation_speed_change(std_msgs::Float32);
private:
    ros::Subscriber vice_change;

    ros::NodeHandle nh_ , ph_;
    // 手柄信息转换器
    explorerJoyExplainer *joy_msg;
    // 接受手柄信息
    ros::Subscriber joy_sub;
    // 持续指令发布
    ros::Timer timer;

    // 基本模式
    int basic_mode_button;
    // 速度调解指令
    int full_speed_button, mid_speed_button, slow_speed_button;
    int speed_front_back, speed_left_right;//未用
    // 副履带简易控制指令
    int front_vice_wheel_up_down, back_vice_wheel_up,back_vice_wheel_down;
    // 副履带速度参数
    double vice_scale;
    double vice_scale_full,vice_scale_mid,vice_scale_slow;
    // 速度参数
    double l_scale, a_scale;
    double l_scale_full,l_scale_mid,l_scale_slow;
    double vice_whell_;
    double a_scale_full,a_scale_mid,a_scale_slow;
    // 副履带平行指令
    int front_vice_wheel_parallel,  back_vice_wheel_parallel;
    // 副履带平行指令发送器
    ros::Publisher vice_wheel_reset_pub;
    // 副履带平行指令
    explorer_msgs::explorer_vice_reset vice_reset_publisher, last_vice_reset_published;
    // 副履带控制模式
    int vice_wheel_mode_button;
    // 车辆前进后退指令
    int linear_front_back, angular_left_right;
    // 车辆运动指令
    geometry_msgs::Twist vel_publisher, last_vel_published;
    // 车辆速度信息发送器
    ros::Publisher vel_pub;
    // 用于在底盘出问题滴滴响时从先启动底盘
    int relive;
    // 副履带调整指令
    /*
    int left_front_vice_wheel_up,   left_front_vice_wheel_down;
    int left_back_vice_wheel_up,    left_back_vice_wheel_down;
    int right_front_vice_wheel_up,  right_front_vice_wheel_down;
    int right_back_vice_wheel_up,   right_back_vice_wheel_down;*/
    int right_front_vice_wheel_up_down;
    int left_front_vice_wheel_up_down;
    int right_back_vice_wheel_up_down;
    int left_back_vice_wheel_up_down;
    // 副履带移动信息发布器
    ros::Publisher vice_wheel_pub;
    // 副履带运动指令
    explorer_msgs::explorer_vice_wheel vice_wheel_publisher, last_vice_wheel_published;

    // 机械臂相关
    // 机械臂控制指令
    int arm_control_button;
    // 机械臂直接控制指令
    int arm_driect_control_button;
    // 机械臂移动信息
    // 解算控制
    explorer_msgs::explorer_moveit_paw last_arm_moveit_published, arm_moveit_publisher;
    //机械臂末端执行器位置移动
    int arm_moveit_up_down;
    int arm_moveit_left_right;
    int arm_paw_moveit_up_down;
    int arm_paw_moveit_left_right;
    int arm_moveit_forward;
    int arm_moveit_back;
    int arm_paw_rotateleft,arm_paw_rotateright;
    // 直接控制
    geometry_msgs::TwistStamped last_arm_driect_published, arm_driect_publisher;  
    int arm_control_forward_back;
    int arm_control_up_down;
    int arm_control_left_right;
    // 机械臂视角转动指令
    int camera_control_up_down;
    int camera_control_left_right;
    // 机械臂复位指令(依照设定,包括视角复位)
    int arm_reset_button;
    // 视角复位指令
    int camera_reset_button;
    // 爪子开合指令
    int paws_control_open_close;
    // 爪子旋转指令
    int paw_turn_left_right;
    // 机械臂移动速度参数
    double arm_scale;
    //机械臂moveit移动参数
    double arm_scale_moveit;
    double arm_scale_moveit1;
    double arm_scale_first;
    // 机械臂视角转动速度参数
    double arm_camera_scale;
    // 机械臂移动信息发布器(带有运动解算)
    ros::Publisher arm_pub;
    // 机械臂移动信息发布器(explorer_arm)
    ros::Publisher arm_driect_pub;
    // 机械臂复位信息发布器
    ros::Publisher reset_pub;
    // 爪子开合信息发布器
    ros::Publisher paw_pub;
    // 爪子开合信息
    std_msgs::Float32 paw_move_msg;

};

#endif // EXPLORER_JOY_CONTROL_H

