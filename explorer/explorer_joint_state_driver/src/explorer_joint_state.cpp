#include "explorer_joint_state.h"
#include <explorer_msgs/explorer_joint.h>

ExplorerJointDriver::ExplorerJointDriver(ros::NodeHandle node)
    :nh_(node)
{
    // joint_names_.push_back("arm1_bearing_joint");
    // joint_names_.push_back("arm2_arm1_joint");
    // joint_names_.push_back("arm3_arm2_joint");
    // joint_names_.push_back("fake_joint");
    // joint_names_.push_back("pt1_arm_joint");
    // joint_names_.push_back("pt2_pt1_joint");
    if (!getNameList(nh_, "/explorer_arm_controller/joints", joint_names_))
    {
        ROS_ERROR("can not get name list");
        return;
    }
    //joint_names_.pop_back();
    joint_names_.push_back("left_down_fin_base_joint");
    joint_names_.push_back("left_down_wheel_base_joint");
    joint_names_.push_back("left_up_fin_base_joint");
    joint_names_.push_back("left_up_wheel_base_joint");
    joint_names_.push_back("right_down_fin_base_joint");
    joint_names_.push_back("right_down_wheel_base_joint");
    joint_names_.push_back("right_up_fin_base_joint");
    joint_names_.push_back("right_up_wheel_base_joint");

    for(int i = 0 ; i < joint_names_.size() ; i++)
    {
        joint_state_pub_[joint_names_[i]] = nh_.advertise<explorer_msgs::explorer_joint>("/"+joint_names_[i]+"/state",10) ;//position就是转过的角度
        joint_position_[joint_names_[i]] = 0.0 ;
    }
    state_sub_ = nh_.subscribe("/joint_states",10 , &ExplorerJointDriver::jointStateSub,this) ;//moveit中的一个topic  接收到的是一个joint_state的数据
}

ExplorerJointDriver::~ExplorerJointDriver()
{

}

void ExplorerJointDriver::jointStateSub(const sensor_msgs::JointStateConstPtr &ptr)
{
    if (ptr->name.size() != joint_names_.size()){
        ROS_ERROR("name list length err");
    }
    for(int i = 0 ; i < ptr->name.size() ; i++)
    {
        if (ptr->name[i].find("cmd")==std::string::npos)joint_position_[ptr->name[i]] = ptr->position.at(i) ;
    }
    pubJointState();
}

void ExplorerJointDriver::pubJointState()
{
    explorer_msgs::explorer_joint joint_msg ;
    for(int i = 0 ; i <  joint_names_.size() ; i++)
    {
        joint_msg.name = joint_names_[i] ;
        ROS_INFO_STREAM(joint_names_[i]) ;
        joint_msg.position = joint_position_[joint_names_[i]] ;//得到规划时的position 现阶段只有position 左负右正
        joint_msg.effort = 0.0 ;
        joint_msg.velocity = 0.0 ;
        joint_state_pub_[joint_names_[i]].publish(joint_msg) ;
    }
}

bool ExplorerJointDriver::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names)
{
    XmlRpc::XmlRpcValue name_list;
    if (!controller_nh.getParam(name_param, name_list))
    {
        ROS_ERROR("can not get the wheel list!!");
        return false;
    }
    if (name_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        if (name_list.size() == 0)
        {
            ROS_ERROR("did not get the wheel name");
            return false;
        }

        for (int i = 0; i < name_list.size(); i++)
        {
            if (name_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("get the error name list");
                return false;
            }
        }
        names.resize(name_list.size());

        for (int i = 0; i < name_list.size(); i++)
        {
            names[i] = static_cast<std::string>(name_list[i]);
        }
    }
    else if (name_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
        names.push_back(name_list);
    }
    else
    {
        ROS_ERROR("the wheel param get error");
        return false;
    }
    return true;
}

int main(int argc , char **argv)
{
    ros::init(argc , argv , "explorer_joint_state_driver")  ;
    ros::NodeHandle node ;
    ExplorerJointDriver explorer_joint_state(node) ;
    ros::spin() ;
    return  0 ;
}