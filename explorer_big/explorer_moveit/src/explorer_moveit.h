#ifndef EXPLORER_MOVEIT_H
#define EXPLORER_MOVEIT_H


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <vector>
#include <explorer_msgs/explorer_moveit_paw.h>
#include <geometry_msgs/Pose.h>
#include <explorer_msgs/explorer_moveit_values.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <explorer_msgs/explorer_reset.h>
#include <geometry_msgs/PoseStamped.h>

class explorer_moveit {


public:
    explorer_moveit(ros::NodeHandle node);
    ~explorer_moveit();
protected:
    void arm_callback(explorer_msgs::explorer_moveit_paw );//接受消息的回调函数
    bool getNameList(ros::NodeHandle controller_nh,  const std::string name_param ,  std::vector<std::string> &names);
    void arm_reset(explorer_msgs::explorer_reset);
private:
    ros::NodeHandle nh_ , ph_;
    std::string reference_frame;
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group;
    const robot_state::JointModelGroup *joint_model_group;
    std::vector< std::string > joint_names,joint_names_;//将机械臂的joint传入
    std::string group_name;
    std::string desc;
    geometry_msgs::PoseStamped current_posi;
    std::vector<double> current_RPY;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    double left_right,up_down,rotate;
    geometry_msgs::Quaternion odom_quat;
    ros::Publisher paw_joint_pub;//发布器
    ros::Subscriber paw_sub,joint_state_sub_,paw_reset;//接收器
    boost::shared_ptr< tf::Transformer >  tf;
    ros::WallDuration  wait_for_servers;//等待时间
    geometry_msgs::Pose target_pose;//目标位置  
    //moveit::planning_interface::MoveGroupInterface::Plan my_plan;//规划
    moveit::planning_interface::MoveItErrorCode success;//判断
    std::vector< double >  group_variable_values,last_group_variable_values;//传输值,同时记录下一个值
    explorer_msgs::explorer_moveit_values joint_values;
};

#endif
