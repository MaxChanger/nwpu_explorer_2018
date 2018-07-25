#include "explorer_joy_control.h"
#include <sstream>
using namespace std;

ExplorerTeleop::ExplorerTeleop():
    ph_("~"),
    // 车辆行驶状态
    // 速度大小调整参数

    basic_mode_button(joy_msg->L1),
    // 速度调解指令
    full_speed_button(joy_msg->button3), mid_speed_button(joy_msg->button2), slow_speed_button(joy_msg->button1),
    //speed_front_back(joy_msg->up_down), speed_left_right(joy_msg->left_right),
    // 副履带简易控制指令
    /*front_vice_wheel_up_down(joy_msg->up_down), back_vice_wheel_up(joy_msg->button1),back_vice_wheel_down(joy_msg->button3),
    right_front_vice_wheel_up_down(joy_msg->right_axes_up_down),
    left_front_vice_wheel_up_down(joy_msg->left_axes_up_down),
    right_back_vice_wheel_up_down(joy_msg->right_axes_up_down),
    left_back_vice_wheel_up_down(joy_msg->left_axes_up_down),*/

    // 副履带平行指令
    /*front_vice_wheel_parallel(joy_msg->button2),  back_vice_wheel_parallel(joy_msg->button4),
    vice_wheel_mode_button(joy_msg->L2),
    // 车辆前进后退指令

    // 用于重新开启底盘
    relive(joy_msg->button4),*/
    linear_front_back(joy_msg->up_down),
    angular_left_right(joy_msg->left_right),
    speed_turn_left(joy_msg->left_axes_left_right),
    
    //机械臂半自动化控制
    /*arm_moveit_up_down(joy_msg->up_down),
    arm_moveit_left_right(joy_msg->left_right),
    //arm_moveit_back(joy_msg->right_axes_up_down),
    arm_moveit_forward(joy_msg->left_axes_up_down),
    arm_paw_moveit_left_right(joy_msg->left_axes_left_right),
    arm_paw_moveit_up_down(joy_msg->right_axes_up_down),
    arm_paw_rotateleft(joy_msg ->right_axes_left_right),
    //arm_paw_rotateright(joy_msg ->button2),

    // 机械臂相关
    // 机械臂控制指令
    
    arm_control_button(joy_msg->R1),
    // 机械臂直接控制指令
    arm_driect_control_button(joy_msg->R2),
    // 机械臂位置移动指令
    //机械臂整体
    

    arm_control_forward_back(joy_msg->left_axes_up_down),//小臂的移动
    arm_control_up_down(joy_msg->up_down),//大臂上下
    arm_control_left_right(joy_msg->left_right),//大臂左右
    // 机械臂视角转动指令
    camera_control_up_down(-joy_msg->right_axes_up_down),//第一个摆动
    camera_control_left_right(joy_msg->left_axes_left_right),//第一个旋转
    // 机械臂复位指令(依照设定,包括视角复位)
    arm_reset_button(joy_msg->right_axes_button),
    // 视角复位指令
    camera_reset_button(joy_msg->left_axes_button),
    // 爪子开合指令
    paws_control_open_close(joy_msg->right_button_left_right),
    // 爪子旋转指令
    paw_turn_left_right(joy_msg->right_axes_left_right),
    // 机械臂移动速度参数
    arm_scale(0.07),
    // 机械臂视角转动速度参数
    arm_camera_scale(0.07),
    arm_scale_moveit(0.004),
    arm_scale_moveit1(0.03),*/
    vice_whell_(2.5),
    arm_scale_first(0.07)
    {
    joy_msg = new autoJoyExplainer();
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 2, &ExplorerTeleop::joyCallback, this);

    vice_change = nh_.subscribe<std_msgs::Float32>("vice_speed_change", 2, &ExplorerTeleop::vice_wheel_speed_change, this);

    vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    /*vice_wheel_reset_pub = nh_.advertise<explorer_msgs::explorer_vice_reset>("explorer_vice_wheel_reset", 1);
    arm_pub = nh_.advertise<explorer_msgs::explorer_moveit_paw>("explorer_moveit_paw", 1);
    arm_driect_pub = nh_.advertise<geometry_msgs::TwistStamped>("explorer_arm_driect", 1);
    vice_wheel_pub = nh_.advertise<explorer_msgs::explorer_vice_wheel>("explorer_vice_wheel", 1);
    reset_pub = nh_.advertise<explorer_msgs::explorer_reset>("explorer_reset", 1);
    paw_pub  = nh_.advertise<std_msgs::Float32>("explorer_paw", 1);*/

    timer = ph_.createTimer(ros::Duration(0.1), boost::bind(&ExplorerTeleop::publishControl, this));
    ph_.param("vice_scale_init", vice_scale,     1.2);
    ph_.param("vice_scale_full", vice_scale_full, 1.6);
    ph_.param("vice_scale_mid", vice_scale_mid, 1.2);
    ph_.param("vice_scale_slow", vice_scale_slow, 0.8);

    ph_.param("l_scale_init",   l_scale,        0.9);//0.3 0.6 1.6
    ph_.param("l_scale_full",   l_scale_full,   1.2);
    ph_.param("l_scale_mid",    l_scale_mid,    0.9);
    ph_.param("l_scale_slow",   l_scale_slow,   0.6);

    ph_.param("a_scale_init",   a_scale,        2.0);// 0.1 0.4 0.6
    ph_.param("a_scale_full",   a_scale_full,   3.0);
    ph_.param("a_scale_mid",    a_scale_mid,    2.0);
    ph_.param("a_scale_slow",   a_scale_slow,   1.5);

}

void ExplorerTeleop::vice_wheel_speed_change(std_msgs::Float32 data) {
    vice_scale = data.data;
}

void ExplorerTeleop::front_speed_change(std_msgs::Float32 data) {
    l_scale = data.data;
}

void ExplorerTeleop::rotation_speed_change(std_msgs::Float32 data) {
    a_scale = data.data;
}

ExplorerTeleop::~ExplorerTeleop() {
    delete joy_msg;
}

void ExplorerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
    joy_msg->getMessage(joy);
    messageClean();
    if(joy_msg->askForButton(basic_mode_button)){
        
            if (joy_msg->askForButton(full_speed_button)) { //速度调节
                l_scale     =    l_scale_full;
                vice_scale  =    vice_scale_full;
                a_scale     =    a_scale_full;
            }

            if (joy_msg->askForButton(mid_speed_button)) {
                l_scale     =    l_scale_mid;
                vice_scale  =    vice_scale_mid;
                a_scale     =    a_scale_mid;
            }

            if (joy_msg->askForButton(slow_speed_button)) {
                l_scale     =    l_scale_slow;
                vice_scale  =    vice_scale_slow;
                a_scale     =    a_scale_slow;
            }
            

            if (std::fabs(joy_msg->askForAxes(linear_front_back)) > 10e-6) {
                vel_publisher.linear.x = joy_msg->askForAxes(linear_front_back) * l_scale;
            }
            if (std::fabs(joy_msg->askForAxes(angular_left_right)) > 10e-6) {
                vel_publisher.angular.z = joy_msg->askForAxes(angular_left_right) * a_scale;
            }
            if (std::fabs(joy_msg->askForAxes(speed_turn_left))> 10e-6){
                vel_publisher.linear.y = joy_msg->askForAxes(speed_turn_left) * l_scale;
            }
        }
    
}

/*
 * 消息发布函数
 * 由createTimer函数生成的计时器保证发送
 * 速率 10hz
 */
void ExplorerTeleop::publishControl() {
    geometry_msgs::Twist vel_empty;
    geometry_msgs::Twist vel_empty1;
    vel_empty1.linear.x =vel_empty1.linear.y =vel_empty1.linear.z=  0;
    vel_empty1.angular.x =vel_empty1.angular.y =vel_empty1.angular.z=  0;
    if (vel_publisher != vel_empty || last_vel_published != vel_empty ) {
        vel_pub.publish(vel_publisher);

        if (vel_publisher == vel_empty) {
            vel_pub.publish(vel_empty1);
            ROS_INFO("publish zero vel");
        }
        last_vel_published = vel_publisher;
        
    }

    explorer_msgs::explorer_vice_wheel vice_empty;

    if (vice_wheel_publisher != vice_empty || last_vice_wheel_published != vice_empty) {
        vice_wheel_pub.publish(vice_wheel_publisher);

        if (vice_wheel_publisher == vice_empty) {
            ROS_INFO("publish zero vice wheel");
        }

        last_vice_wheel_published = vice_wheel_publisher;
    }

    explorer_msgs::explorer_moveit_paw arm_empty;
    geometry_msgs::TwistStamped arm_empty1;
   /* if (arm_moveit_publisher.x != arm_empty.x || arm_moveit_publisher.y != arm_empty.y||
        arm_moveit_publisher.up_down != arm_empty.up_down || arm_moveit_publisher.left_right != arm_empty.left_right||
        arm_moveit_publisher.rotate != arm_empty.rotate) {*/
    if (arm_moveit_publisher != arm_empty || last_arm_moveit_published != arm_empty) {    
        arm_pub.publish(arm_moveit_publisher);

        if (arm_moveit_publisher == arm_empty) {
            ROS_INFO("publish arm zero0.0.0.0.");
        }

        last_arm_moveit_published = arm_moveit_publisher;
    }

    if (arm_driect_publisher.twist != arm_empty1.twist || last_arm_driect_published.twist != arm_empty1.twist) {
        arm_driect_publisher.header.stamp = ros::Time::now();
        arm_driect_pub.publish(arm_driect_publisher);

        if (arm_driect_publisher == arm_empty1) {
            ROS_INFO("publish driect arm zero");
        }
        ROS_INFO("publish driect arm zero");
        last_arm_driect_published = arm_driect_publisher;
    }

    std_msgs::Float32 float_empty;

    if (paw_move_msg != float_empty) {
        paw_pub.publish(paw_move_msg);
    }

    explorer_msgs::explorer_vice_reset reset_empty;

    if (vice_reset_publisher != reset_empty || last_vice_reset_published != reset_empty) {
        vice_wheel_reset_pub.publish(vice_reset_publisher);

        if (vice_reset_publisher == reset_empty) {
            ROS_INFO("vice wheel reset stop");
        }

        last_vice_reset_published = vice_reset_publisher;
    }
}

/*
 * 用于清空所有的数据
 * 用于收到新的指令之时
 */
void ExplorerTeleop::messageClean() {
    geometry_msgs::Twist vel_empty;
    this->vel_publisher = vel_empty;
}
