//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
/**
 * 
controller的几个作用：
1.调用timerPlanExploration 请求(call) exploration_node给他一个探索路径的回复（并且广播了出去）  
  然后node调用planner里边的doExploration
  返回的答案存在了srv_exploration_plan里面  
  然后又调用了follower里边的setplan函数
  setplan函数用到了刚才返回的结果作为参数  然后setPlan又调用follower里边的transformGlobalPlan函数 结果存在了global_plan_中
  最后global_plan_经过下面computeVelocityCommands计算速度的时候调用了
  
2.调用timerCmdVelGeneration 然后调用timerCmdVelGeneration调用follower 里边的 computeVelocityCommands（计算下一步的速度）
computeVelocityCommands用了global_plan_的数据 最后生成要发布的速度存在了twist里边 发给底盘
 */

#include <ros/ros.h>
#include <hector_path_follower/hector_path_follower.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
////
#include <cmath>
const float vel_factor = 0.18;
////

class SimpleExplorationController
{
public:
  ////SimpleExplorationController():
  SimpleExplorationController(): nh_private("~")
  {
    ros::NodeHandle nh;
    ///////////////////////////////////////////
    nh_private.param("p_linear_vel", p_linear_vel, 0.5);
    nh_private.param("p_rot_vel", p_rot_vel, 0.35);
    //////////////////////////////////////////
    
    exploration_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");
    ///这里是客户端 发送请求到服务器exploration_node要求给他一个探索路径的回复 然后他调用了doExploration 得到的应该是全局规划路径 之后通过该node转化为速度cmd_vel发送给地盘

    path_follower_.initialize(&tfl_);
    //timer定时器  定时规划路径和定时发布转化

    /**
     * 
     * 
     * 
     * 
     * 
     * */
    
    exploration_plan_generation_timer_ = nh.createTimer(ros::Duration(2.0), &SimpleExplorationController::timerPlanExploration, this, false );
    //勘探计划生成时间    //我的是15.0 白色工控机是0.3
    
    cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration, this, false );

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/explorer_drive_controller/cmd_vel", 10);
    ///发布话题cmd_vel 由底盘接受 这一项可以更改要根据底盘那面设定的话题来确定
  }

  void timerPlanExploration(const ros::TimerEvent& e)
  {
    ////
    if (!isReached) //如果没有到到目的地
      return;
    ////

    hector_nav_msgs::GetRobotTrajectory srv_exploration_plan;

    if (exploration_plan_service_client_.call(srv_exploration_plan)){
      //srv_exploration_plan.response就是得到的全局路径
      //调用node的explorationServiceCallback    hector_nav_msgs::GetRobotTrajectory::Response &res
      
      ROS_INFO("Generated exploration path with %u poses", (unsigned int)srv_exploration_plan.response.trajectory.poses.size());
      
      //// path_follower_.setPlan(srv_exploration_plan.response.trajectory.poses);
      if(!path_follower_.setPlan(srv_exploration_plan.response.trajectory.poses))//以global_plan传入
      {
        /**
         * 设置Plan失败了 之后发送各方速度为0
         * srv_exploration_plan.response.trajectory.poses以静态变量传进去 并不能被修改
         * geometry_msgs/PoseStamped[] poses
         * 拿到位置 交给follower的setPlan去设置计划 setPlan又调用follower里边的
         * transformGlobalPlan函数 结果存在了global_plan_
         * global_plan_  是得到的“局部路径规划“
         * 最后的global_plan_经过下面computeVelocityCommands计算速度的时候
         * 调用了global_plan_的数值
         */
        ROS_ERROR("In controller | path_follower.setPlan failed!");
        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
        vel_pub_.publish(twist);
      }
    }
    else
    {
      ROS_WARN("Service call for exploration service failed");
    }
  }

  void timerCmdVelGeneration(const ros::TimerEvent& e)
  {
   
    geometry_msgs::Twist twist;//定义一个类型

    path_follower_.computeVelocityCommands(twist);
    /**
     * 把计算得到的东西存到了twist里边    
     * ::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
     * 直接调用follower类里边的方法（函数）用了自己类里边的一些值，比如调用这个函数用到了上边
     * follower”计算速度的命令“ 通过调用该函数 获得到x y方向速度和方向角
     */
    /**
     * @brief 在前边有
     * nh_private.param("p_linear_vel", p_linear_vel, 0.5);
     * nh_private.param("p_rot_vel", p_rot_vel, 0.35);
     */
    
    twist.linear.x *= p_linear_vel * 7;
    twist.linear.y *= p_linear_vel * 7;
    twist.angular.z *= p_rot_vel * 4;
    //ROS_WARN("%f \t %f \t %f",fabs(twist.linear.x),fabs(twist.linear.y),fabs(twist.linear.z));
    if (fabs(twist.linear.x) < 1e-6 && fabs(twist.angular.z) < 1e-6 && fabs(twist.angular.y) < 1e-6)
    {	
      isReached = true;    //判断是否到达目标点 在满足上边的几个调教的情况下 判断为到达
      return;
    }
    if (fabs(twist.angular.z) > M_PI)
    {
      if (twist.angular.z < 0)
        twist.angular.z += 2 * M_PI;
      else
        twist.angular.z -= 2 * M_PI;
    }
    
    twist.linear.x *= vel_factor * 0.9;
    twist.linear.y *= vel_factor * 0.9;
    twist.angular.z *= vel_factor * 0.9;//调整大小
    ROS_ERROR("[hector_exploration_controller] vel pub %lf %lf ==>%lf", twist.linear.x, twist.linear.y ,twist.angular.z);
    vel_pub_.publish(twist);//发送给底盘
  }


protected:
  ros::ServiceClient exploration_plan_service_client_;

  ros::Publisher vel_pub_;

  tf::TransformListener tfl_;

  pose_follower::HectorPathFollower path_follower_;

  ////
  ros::NodeHandle nh_private;
  bool isReached;
  double p_linear_vel, p_rot_vel;
  ////

  ros::Timer exploration_plan_generation_timer_;//勘探计划生成时间
  ros::Timer cmd_vel_generator_timer_;//定时解算速度（实时）
};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SimpleExplorationController ec;

  ros::spin();

  return 0;
}
