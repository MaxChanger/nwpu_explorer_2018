#ifndef EXPLORER_ARM_CONTROLLER_H
#define EXPLORER_ARM_CONTROLLER_H
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <explorer_msgs/explorer_arm.h>
#include <explorer_msgs/explorer_reset.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>
#include <queue>
#include <map>
#include <string>
#include "joint_controller.hpp"
#include <moveit_msgs/RobotTrajectory.h>
#include <explorer_msgs/explorer_moveit_values.h>
#include <explorer_msgs/explorer_low_level_data.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
namespace explorer_arm_controller {
struct position {
    std::vector<double> Position;
    ros::Time time;
    position():
        time(0.0) {}
};
class ArmController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
    ArmController();
    ~ArmController();
    // 初始化函数 是 controller 的构造要求
    // 返回值为是否构造成功
    bool init(hardware_interface::PositionJointInterface *joint_handle_/*form hardware*/, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    // start 与 stop 函数采用 Controller 默认的函数,即置空

    // 更新函数,每次机械臂的位置数据更新都在这个函数里进行
    void update(const ros::Time &time, const ros::Duration &period);
private:
    // 获取机械臂名称列表
    bool getNameList(ros::NodeHandle controller_nh,  const std::string name_param ,  std::vector<std::string> &names);

    //void jointStateSub(const explorer_msgs::explorer_arm & ptr);
    void jointStateSub(const geometry_msgs::TwistStamped &msg);

    void resetStateSub(const explorer_msgs::explorer_reset &ptr);

    void pawStateSub(const std_msgs::Float32 &ptr);

    void moveitSub(const explorer_msgs::explorer_moveit_values &ptr);
    void yuntaisub(explorer_msgs::explorer_low_level_data msg);
    void pushArmResetPose();
private:
    std::vector<std::string> arm_name;              // 机械臂名称列表
    std::map<std::string, joint*> joint_map;
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Subscriber yuntai_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber reset_sub_;
    ros::Subscriber paw_sub_;
    ros::Subscriber moveit_sub_;
    geometry_msgs::Quaternion odom_quat;
    sensor_msgs::Imu imu_;
    ros::Time front_time;                           // 上一次更新时间
    int now_moveit_pose;
    std::queue<std::vector<std::pair<std::string, double> > > reset_queue; // 机械臂复原任务队列
};
//这句话将这个    类设置为插件
PLUGINLIB_EXPORT_CLASS(explorer_arm_controller::ArmController, controller_interface::ControllerBase);
}

#endif  //EXPLORER_ARM_CONTROLLER_HH