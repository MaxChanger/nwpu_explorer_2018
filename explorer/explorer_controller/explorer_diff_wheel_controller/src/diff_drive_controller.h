#ifndef DIFF_DRIVE_CONTROLLER_H
#define DIFF_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <string>
#include <map>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
using namespace std;
//using namespace controller_interface;

struct Commands {
    double linear_x_vel;
    double linear_y_vel;
    double angular_z_vel;
    double linear_z_vel;
    ros::Time current_time;

    Commands(): linear_x_vel(0.0), angular_z_vel(0.0), current_time(0.0),linear_z_vel(0.0) {}
};

namespace explorer_diff_drive_controller {
class DiffDriveController: public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
public:
    DiffDriveController();
    ~DiffDriveController();
    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);
    void update(const ros::Time &time, const ros::Duration &period);
    void starting(const ros::Time &time);
    void stopping(const ros::Time &time);
protected:
private:
    void velocityCallBack(const geometry_msgs::TwistConstPtr &ptr);
    void brake();
    bool getWheelNameLists(ros::NodeHandle controller_nh,  const string wheel_param ,  vector<string> &wheel_names);
private:
    double wheel_radius_;
    double wheel_radius_multiplier_;
    double wheel_separation_;
    double cmd_vel_timeout_;
    double car_length;
    double car_width;
    double raw_param;

    vector<string> left_wheel_name_;
    vector<string> right_wheel_name_;

    ros::Subscriber cmd_vel_sub_;

    map<string , hardware_interface::JointHandle> left_wheel_joint_ ;
    map<string , hardware_interface::JointHandle> right_wheel_joint_;

    realtime_tools::RealtimeBuffer <Commands> realtime_buffer_;
    Commands current_command_;
    Commands last_command_;
};
PLUGINLIB_EXPORT_CLASS(explorer_diff_drive_controller::DiffDriveController , controller_interface::ControllerBase);
}


#endif // DIFF_DRIVE_CONTROLLER_H

