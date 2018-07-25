#ifndef VICE_WHEEL_CONTROLLER_H
#define VICE_WHEEL_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <string>
#include <vector>
#include <map>
#include <math.h>
#include <explorer_msgs/explorer_vice_wheel.h>
#include <explorer_msgs/explorer_vice_reset.h>

using namespace std ;

namespace explorer_vice_wheel_controller {

struct Commands {
    float front_left_vice_angular, front_right_vice_angular;
    float back_left_vice_angular, back_right_vice_angular;
    ros::Time time;
    Commands() :    front_left_vice_angular(0.0), front_right_vice_angular(0.0),
        back_left_vice_angular(0.0), back_right_vice_angular(0.0),
        time(0.0) {}
};

class ViceWheelController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
    ViceWheelController() ;
    ~ViceWheelController() ;
    bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) ;
    void starting(const ros::Time &time) ;
    void stopping(const ros::Time &time) ;
    void update(const ros::Time &time, const ros::Duration &period);
protected:
private:
    vector<string> front_vice_wheel_names_  ;
    vector<string> back_vice_wheel_names_ ;

    double timeout_ ;
    double angular_multiplier_ ;
    double max_speed;

    map<string, hardware_interface::JointHandle> front_vice_wheel_handles_;
    map<string , hardware_interface::JointHandle> back_vice_wheel_handles_;
    map<string, double>command_map;
    realtime_tools::RealtimeBuffer<Commands> realtime_buffer_ ;
    Commands current_command_ ;
    Commands last_command_ ;
    explorer_msgs::explorer_vice_reset reset_msg;

    ros::Subscriber vice_wheel_sub_ ,vice_reset_sub;
private:
    void viceWheelCallBack(const explorer_msgs::explorer_vice_wheelConstPtr &ptr) ;
    void resetCommandCallBack(const explorer_msgs::explorer_vice_reset &msg);
    void brake() ;
    bool getViceWheelNameList(ros::NodeHandle controller_nh,  const string vice_wheel_param ,  vector<string> &vice_wheel_names) ;
};
PLUGINLIB_EXPORT_CLASS(explorer_vice_wheel_controller::ViceWheelController, controller_interface::ControllerBase) ;
}

#endif // VICE_WHEEL_CONTROLLER_H
//https://github.com/ros-controls/ros_control/blob/81ed9edf2a399c1aae33fc5b603c2215dde5ffbe/hardware_interface/include/hardware_interface/joint_command_interface.h