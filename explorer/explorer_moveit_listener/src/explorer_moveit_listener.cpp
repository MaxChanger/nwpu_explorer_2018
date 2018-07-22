#include <ros/ros.h>  
#include <actionlib/server/simple_action_server.h>  
#include <control_msgs/FollowJointTrajectoryAction.h>  
#include <trajectory_msgs/JointTrajectory.h>  
#include <std_msgs/Float64.h>  
#include <iostream>  
#include <vector>  
#include <string>  
#include <sensor_msgs/JointState.h>  
#include <map>  
#include <explorer_msgs/explorer_moveit_values.h> 
using namespace std ;  
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;  //用于轨迹的server
  
class RobotTrajectoryFollower  
{  
protected:  
  
  ros::NodeHandle nh_;  
  std::string action_name_;  
  
  ros::Publisher joint_pub ; //向rviz发布 
  ros::Publisher joint_vales_pub ; //向下位机发送 
  sensor_msgs::JointState joint_state;  //节点的状态
  explorer_msgs::explorer_moveit_values joint_values1,joint_values2;//向下发送的joint_values,需要两个值，向下传输的是两个值相减
  control_msgs::FollowJointTrajectoryResult result_;  
  control_msgs::FollowJointTrajectoryActionGoal agoal_;  
  control_msgs::FollowJointTrajectoryActionFeedback afeedback_;  
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  std::vector<double> joint_posi1;
  int arm_name[6];
public:  
  map< string, int > MotoName_Id;  
  RobotTrajectoryFollower(std::string name) :  
    nh_("~"),  
    as_(nh_, name,boost::bind(&RobotTrajectoryFollower::goalCB, this, _1),  false),  
    action_name_(name)  
  {  
    
    
    int m =0;
    for(;m<7;m++){
      joint_values1.names.push_back(m+1);
      joint_values2.names.push_back(m+1);
      joint_values1.values.push_back(0);
      joint_values2.values.push_back(0);
    }

    ROS_INFO("GET THE NAME");             //grroup无法更新全部的消息
    joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);  //向rviz中发布消息来更新模型的状态
    joint_vales_pub = nh_.advertise<explorer_msgs::explorer_moveit_values>("explorer_moveit_joint", 50);  //向下传递
  
    as_.registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));  
  
    as_.start();  
  }  
  
  ~RobotTrajectoryFollower(void)//Destructor  
  {  
  }  
  // 发送至rviz中的 
  void setJointStateName(std::vector<std::string> joint_names){  
    joint_state.name.resize(joint_names.size());  
    joint_state.name.assign(joint_names.begin(), joint_names.end());    
  } 

  void setJointStatePosition(std::vector<double> joint_posi){  
        joint_state.position.resize(joint_posi.size());  
        joint_state.position.assign(joint_posi.begin(), joint_posi.end());
  }

  void publishJointState(){  
    joint_state.header.stamp = ros::Time::now();  
    joint_pub.publish(joint_state);  
  }  
  //向下发送可以直接发送至arm_controller中
  //需要修改
  //此处用于将explorer_msgs::explorer_moveit_values类型额消息发送下去
  void joint_states_pub(std::vector<double> joint_posi){ 
    
    int i=0;
    int joint_posi_length = joint_posi.size();
    for(i= 0;i<joint_posi_length;i++){
      joint_values1.values[i] = (float) joint_posi[i] - joint_values2.values[i];
      joint_values2.values[i] = joint_posi[i];
    }
    joint_vales_pub.publish(joint_values1);
  } 



  void goalCB(const control_msgs::FollowJointTrajectoryGoalConstPtr msg)  
  {  
    std::vector<std::string> joint_names=(*msg).trajectory.joint_names;  
    setJointStateName( joint_names);  
  
    std::vector<trajectory_msgs::JointTrajectoryPoint> points = (*msg).trajectory.points;//轨迹上的位置  
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator pointsit;
    ros::Rate rate(10);//10hz  如何更改发出的速度值
    //size_t t=points.size();      
    for ( pointsit = points.begin(); pointsit != points.end(); pointsit++){   
      joint_states_pub((*pointsit).positions);
      //wait  
      rate.sleep();
      //then update joinstates an publish  
      setJointStatePosition((*pointsit).positions); 
      publishJointState();  
    } 
    if(as_.isActive())as_.setSucceeded(); 
  }  
  
  void preemptCB()  
  {  
    ROS_INFO("%s: Preempted", action_name_.c_str());  
  
    if(as_.isActive()){  
        as_.setPreempted();  
    }  
  } 
  Server as_  ;   
};  
  
int main(int argc, char** argv)  
{  
  
  ros::init(argc, argv, "explorer_moveit_listener");  
  RobotTrajectoryFollower RobotTrajectoryFollower("/explorer_arm_controllers/follow_joint_trajectory");   
  ROS_INFO("------------- joint controller is running.Good Luck.");  
  ros::spin();  
  return 0;   
}

