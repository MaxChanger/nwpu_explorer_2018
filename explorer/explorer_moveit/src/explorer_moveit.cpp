#include "explorer_moveit.h"
#include <sstream>

explorer_moveit::explorer_moveit(ros::NodeHandle node):
  nh_(node),PLANNING_GROUP("explorer_arm"),move_group(PLANNING_GROUP)

{
    // reference_frame = "base_link";
    // move_group.setPoseReferenceFrame(reference_frame);
     current_posi = move_group.getCurrentPose("paw_rotation_link");
    // current_RPY = move_group.getCurrentRPY("paw_rotation_link");
    /*
    此为直接从rviz读取
    */
    // target_pose.position.x =  current_posi.pose.position.x;
    // target_pose.position.y = current_posi.pose.position.y;
    // target_pose.position.z = current_posi.pose.position.z;
    // target_pose.orientation.x = current_posi.pose.orientation.x;
    // target_pose.orientation.y = current_posi.pose.orientation.y;
    // target_pose.orientation.z = current_posi.pose.orientation.z;
    // target_pose.orientation.w = current_posi.pose.orientation.w;
    // left_right = current_RPY[2];
    // up_down = current_RPY[1];
    // rotate = current_RPY[0];
    /*
    此为直接从commender看的数据
    */
    target_pose.position.x = 0.281500091384; 
    target_pose.position.y = -0.0121031512449;
    target_pose.position.z = 0.402651647112;
    target_pose.orientation.x = -0.984601685;
    target_pose.orientation.y = 0.0128524297469;
    target_pose.orientation.z = 0.0848902778232;
    target_pose.orientation.w = 0.152275991791;
    left_right = 0.0005526015253083062;
    up_down = 0.17192618652724706;
    rotate = -2.8346614822056826;
    ROS_ERROR_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    move_group.setPlannerId("RRTConfigDefault");
    paw_sub = nh_.subscribe<explorer_msgs::explorer_moveit_paw>("explorer_moveit_paw", 2, &explorer_moveit::arm_callback, this);
  }
explorer_moveit::~explorer_moveit(){}
//绕x为旋转
//绕y为上下
//绕z为左右
void explorer_moveit::arm_callback(explorer_msgs::explorer_moveit_paw pawptr){
    //先进行初始位置的再次标定，将两个模式统一
    
    // target_pose.position.x =  current_posi.pose.position.x + pawptr.x;
    // target_pose.position.y = current_posi.pose.position.y + pawptr.y;
    // target_pose.position.z = current_posi.pose.position.z + pawptr.z;
    // left_right = current_RPY[2] + pawptr.left_right;
    // up_down = current_RPY[1] + pawptr.up_down;
    // rotate = current_RPY[0] + pawptr.rotate;


    target_pose.position.x += pawptr.x; 
    target_pose.position.y += pawptr.y;
    target_pose.position.z += pawptr.z;
    left_right +=pawptr.left_right;
    up_down += pawptr.up_down;
    rotate += pawptr.rotate;

    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(rotate,up_down,left_right);
    ROS_ERROR_STREAM("  w:"<<odom_quat.w << "  x:"<<odom_quat.x<<"  y:"<<odom_quat.y << "  z:"<<odom_quat.z);
    target_pose.orientation.w = (double)odom_quat.w;
    target_pose.orientation.x = (double)odom_quat.x;
    target_pose.orientation.y = (double)odom_quat.y;
    target_pose.orientation.z = (double)odom_quat.z;

    move_group.setPoseTarget(target_pose);
      move_group.asyncMove ();    
}

int main (int argc,char **argv){
  ros::init(argc, argv, "explorer_moveit");
  ros::NodeHandle nodehandle;  
  explorer_moveit explorer_moveit(nodehandle);  
  ros::spin();
}
