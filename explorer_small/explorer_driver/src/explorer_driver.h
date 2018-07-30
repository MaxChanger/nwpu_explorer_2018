#ifndef EXPLORER_DRIVER_H
#define EXPLORER_DRIVER_H

#include <ros/ros.h>
#include <cstdlib>
#include <explorer_msgs/explorer_agreement.h>
#include <explorer_msgs/explorer_message.h>
#include <vector>
#include <std_msgs/String.h>

using namespace std;

typedef explorer_msgs::explorer_agreement AgreementMessage;
typedef vector<float> Message;

class RobotDriver {
public:
    RobotDriver(ros::NodeHandle nh_);
    ~ RobotDriver();
protected:

private:
    //void shutdowndriver(const std_msgs::String::ConstPtr& ptr);
    void robotMsgSub(const explorer_msgs::explorer_messageConstPtr &ptr);
    AgreementMessage messageToAgreement(int component_id , Message data);
    ros::NodeHandle robot_driver_node__;
    ros::Publisher robot_serial_data_pub__;
    ros::Subscriber robot_driver_sub__;
};

#endif // EXPLORER_DRIVER_HH