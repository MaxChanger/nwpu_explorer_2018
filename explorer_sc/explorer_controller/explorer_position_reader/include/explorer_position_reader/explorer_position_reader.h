#ifndef EXPLORER_ARM_CONTROLLER_H
#define EXPLORER_ARM_CONTROLLER_H
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <explorer_/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TwistStamped.h>
#include <explorer_robot_hardware/explorer_position_reader_interface.hpp>
#include <explorer_msgs/explorer_low_level_data.h>
namespace explorer_position_reader {

class positionReader : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
    positionReader();
    ~positionReader();
    // 初始化函数 是 controller 的构造要求
    // 返回值为是否构造成功
    bool init(explorer_position_reader::PosionReaderInterface *posion_interface/*form hardware*/, ros::NodeHandle &, ros::NodeHandle &);
    // start 与 stop 函数采用 Controller 默认的函数,即置空

    // 更新函数,每次机械臂的位置数据更新都在这个函数里进行
    void update(const ros::Time &, const ros::Duration &);
private:

    //void jointStateSub(const explorer_msgs::explorer_arm & ptr);
    void jointStateSub(const ::TwistStamped &msg);

private:

    ros::NodeHandle nh;

};
//这句话将这个类设置为插件
PLUGINLIB_EXPORT_CLASS(explorer_position_reader::positionReader, controller_interface::ControllerBase);
}

#endif  //EXPLORER_ARM_CONTROLLER_HH