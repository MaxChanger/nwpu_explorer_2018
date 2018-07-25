#ifndef EXPLORER_JOINT_STATE_H
#define EXPLORER_JOINT_STATE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <map>
#include <string>

using namespace std ;

class ExplorerJointDriver
{
public:
    ExplorerJointDriver(ros::NodeHandle node) ;
    ~ExplorerJointDriver() ;
private:
    ros::NodeHandle nh_ ;
    ros::Subscriber state_sub_ ;
    map<string , ros::Publisher> joint_state_pub_ ;
    vector<string> joint_names_ ;
    map<string,double> joint_position_ ;
private:
    void pubJointState() ;
    void jointStateSub(const sensor_msgs::JointStateConstPtr &ptr) ;
    // 获取机械臂名称列表
    bool getNameList(ros::NodeHandle controller_nh,  const std::string name_param ,  std::vector<std::string> &names);
};

#endif // EXPLORER_JOINT_STATE_H
