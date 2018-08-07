#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <explorer_msgs/explorer_joint.h>
#include <explorer_msgs/explorer_message.h>
#include <explorer_msgs/explorer_low_level_data.h>
#include <ros/callback_queue.h>
#include <explorer_robot_hardware/explorer_position_reader_interface.hpp>

class ExplorerHardware : public hardware_interface::RobotHW {
public:
    ExplorerHardware(ros::NodeHandle node) ;
    ~ExplorerHardware();
    void read(ros::Time current_time , ros::Duration period);
    void write(ros::Time current_time , ros::Duration period);
    bool start();
    void stop();
    ros::Duration getPeriod();
    ros::Time getCurrentTime();
    ros::CallbackQueue *getCallBackQuequ();
    float getFrep();
    bool getNameList(ros::NodeHandle,  const ::std::string,  ::std::vector<::std::string> &);
protected:

private:

    // 用于被explorer_position_reader调用，当收到电子层数据时更新位姿数据
    ::explorer_interface::PosionReaderInterface posion_reader_interface;

    // 用于被joint_state_controller调用,注意这个只能读取不能调用
    ::hardware_interface::JointStateInterface joint_state_interface_;

    // 用于被arm_controller vice_wheel_controller 调用
    ::hardware_interface::PositionJointInterface joint_position_interface_;

    //被diff_wheel_controller 调用
    ::hardware_interface::VelocityJointInterface joint_vel_interface_;

    ::ros::NodeHandle nh_;
    ::ros::NodeHandle nh_private;
    ::ros::Publisher robot_drive_pub_;
    ::ros::Subscriber down_vice_wheel_pos_sub, up_vice_wheel_pos_sub;
    double left_up_vice_wheel_init;
    double left_down_vice_wheel_init;
    double right_up_vice_wheel_init;
    double right_down_vice_wheel_init;

    static const int up_vice_id = 0x12, down_vice_id = 0x11;

    ::sensor_msgs::JointStateConstPtr current_joint_state_ptr_;

    ::ros::CallbackQueue callback_quequ_;

    ::std::vector<::std::string> base_joint_name_;        //base wheel joint name
    ::std::vector<::std::string> vice_wheel_joint_name_;  //vice wheel joint name
    ::std::vector<::std::string> arm_joint_name_;         //arm joint name

    ::std::map<::std::string , ros::Subscriber> joint_state_sub_;

    ::std::map<::std::string , double> joint_pose_;
    ::std::map<::std::string , double> base_wheel_pos_;
    ::std::map<::std::string , double> base_wheel_cmd_;
    ::std::map<::std::string , double> base_wheel_eff_;
    ::std::map<::std::string , double> base_wheel_vel_;

    ::std::map<::std::string , double> vice_wheel_pos_;
    ::std::map<::std::string , double> vice_wheel_cmd_;
    ::std::map<::std::string , double> vice_wheel_eff_;
    ::std::map<::std::string , double> vice_wheel_vel_;

    ::std::map<::std::string , double> arm_joint_pos_;//传向rviz的角度位置信息
    ::std::map<::std::string , double> arm_joint_cmd_;//从controller传来的命令信息
    ::std::map<::std::string , double> arm_joint_eff_;
    ::std::map<::std::string , double> arm_joint_vel_;
    ::std::map<::std::string , double> arm_joint_off_pos_;    //the initialize position of the robot arm

    ::std::map<::std::string , double> vice_wheel_off_pos_;   //the initialize position of the robot vice wheel


    double last_front_vice_wheel_angular_;
    double last_back_vice_wheel_angular_;
    double last_left_wheel_vel_;
    double last_right_wheel_vel_;
    float frep_;

    ::boost::shared_ptr<ros::AsyncSpinner> spinner_;

private:
    void pub_vel_cmd();
    void pub_vice_wheel_cmd();
    void pub_arm_joint_cmd() ;
    void getFrontViceWheelPos(const ::explorer_msgs::explorer_low_level_data::ConstPtr &msg);
    void getBackViceWheelPos(const ::explorer_msgs::explorer_low_level_data::ConstPtr &msg);
    void jointStateCallBack(const explorer_msgs::explorer_jointConstPtr &ptr);
};



#endif // ROBOT_HARDWARE_H


