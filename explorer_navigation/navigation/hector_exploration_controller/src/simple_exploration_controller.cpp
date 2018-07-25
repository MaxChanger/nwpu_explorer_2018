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
#include <cmath>

const float vel_factor = 0.18;
int call_timerCmdVelGeneration_counter = 0;

class SimpleExplorationController
{

public:
  SimpleExplorationController(): nh_private("~")
  {
    ROS_WARN("Begin in SimpleExplorationController");
    ros::NodeHandle nh;
    
    nh_private.param("p_linear_vel", p_linear_vel, 0.5);
    nh_private.param("p_rot_vel", p_rot_vel, 0.35);
    isReached = false;
    
    
    exploration_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");
    // 这里是客户端 发送请求到服务器exploration_node要求给他一个探索路径的回复 然后他调用了doExploration 
    // 得到的应该是全局规划路径 之后通过该node转化为速度cmd_vel发送给地盘

    path_follower_.initialize(&tfl_);    //timer定时器  定时规划路径和定时发布转化
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/explorer_drive_controller/cmd_vel", 10); //底盘接受速度的话题



    // 机器原地 左方向 转圈 为了生成更好costmap 
    int i = 0;
    int flag = 1;
    for(i = 0 ; i < 25 ; i++){   // 改成25  
        geometry_msgs::Twist twist;//定义一个类型
        if(i%2 == 0){
          //flag = -flag;
        }
        twist.linear.x   =  0 ;
        twist.linear.y   =  0 ;
        twist.angular.z  =  0.2 * flag ;//调整大小
        
        ROS_INFO("[Move By Itself] velocity pub x:%.2lf y:%.2lf z:%.2lf", twist.linear.x, twist.linear.y ,twist.angular.z);
        vel_pub_.publish(twist);//发送给底盘
        sleep(1);
    }//end loop
   


    isReached = true;//判断是否到达goals

    exploration_plan_generation_timer_ = nh.createTimer(ros::Duration(2.0), &SimpleExplorationController::timerPlanExploration, this, false );//每过多少秒调用一次 生产plan的函数  在加上isReach之后其实多少并没有多大影响  //目前是7.0 白色工控机是0.3

    // sun_cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(10.0), &SimpleExplorationController::timerCmd_Sun, this, false );
    // 每10秒钟回调一次 这个是每隔一定时间之后 发送一个速度 让他动一下

    cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration, this, false );
    //ros::Duration(0.1)则回调将按0.1秒安排    

  }

  void timerPlanExploration(const ros::TimerEvent& e)
  {
    ROS_WARN("In Generate Plan Exploration");
    ROS_WARN("isReached :%d", isReached);

    if(!isReached){ // 当isReached = 0 时，即没有到达goal的时候 就return 不进行新的路径规划
      ROS_WARN("[Gengerate Plan]Because don't reached goal ------------> Return\n\n");
        return;
    }else{
      ROS_ERROR("Reach Goal");
    }

    hector_nav_msgs::GetRobotTrajectory srv_exploration_plan;

    if (exploration_plan_service_client_.call(srv_exploration_plan)){
      //srv_exploration_plan.response就是得到的全局路径
      //调用node的explorationServiceCallback    hector_nav_msgs::GetRobotTrajectory::Response &res
      
      ROS_INFO("Generated exploration path with %u poses", (unsigned int)srv_exploration_plan.response.trajectory.poses.size());
      
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
    
    isReached = false;
  }

  void timerCmdVelGeneration(const ros::TimerEvent& e)
  {
    ROS_WARN("In timer CmdVel Generation");
    ROS_WARN("isReached :%d", isReached);

    // if(isReached){
    //   ROS_ERROR("return in timer CmdVel Generation");
    //   return;
    // }
    call_timerCmdVelGeneration_counter ++;

    geometry_msgs::Twist twist; // 定义一个类型

    path_follower_.computeVelocityCommands(twist,isReached);
    
    if ( !(twist.linear.x == 0 && twist.linear.y == 0 && twist.angular.z == 0) 
        && fabs(twist.linear.x) < 0.1 && fabs(twist.linear.y) < 0.1 && fabs(twist.angular.z) < 0.3 ) // 这个条件参考的是follower里边的条件
    {	
      isReached = true;    //判断是否到达目标点 在满足上边的几个条件的情况下 判断为到达
      return;
    }
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

    if (fabs(twist.angular.z) > M_PI)
    {
      if (twist.angular.z < 0)
        twist.angular.z += 2 * M_PI;
      else
        twist.angular.z -= 2 * M_PI;
    }
    
    // twist.linear.x   *= vel_factor * 1.5 ;
    // twist.linear.y   *= vel_factor * 0   ; // 大救援没有平移 故取消这个方向的速度
    // twist.angular.z  *= vel_factor * 7   ; // 调整速度大小
    
     twist.linear.x   *= vel_factor * 1.2 ;
     twist.linear.y   *= vel_factor * 0.9 ;
     twist.angular.z  *= vel_factor * 1.0 ;
    
    ROS_ERROR("[hector_exploration_controller] Velocity pub %lf %lf ==>%lf", twist.linear.x, twist.linear.y ,twist.angular.z);

    if(fabs(twist.linear.x) > 0.5){
      twist.linear.x = 0.5;
    }

    vel_pub_.publish(twist);//发送给底盘
  }






  void timerCmd_Sun(const ros::TimerEvent& e)
  {
     ROS_WARN("timerCmd_Sun");
    geometry_msgs::Twist twist; //定义一个类型

    twist.linear.x   =  0.5 ;   //小救援给这个速度有点大
    twist.linear.y   =  0   ;
    twist.angular.z  =  0   ;   //调整大小
        
    ROS_ERROR("[timerCmd_Sun] Velocity pub %.2lf %.2lf ==>%.2lf", twist.linear.x, twist.linear.y ,twist.angular.z);
    
    vel_pub_.publish(twist);//发送给底盘
    sleep(1);
  }


protected:
  ros::ServiceClient exploration_plan_service_client_;
  ros::Publisher vel_pub_;
  tf::TransformListener tfl_;
  pose_follower::HectorPathFollower path_follower_;

  ros::NodeHandle nh_private;
  bool isReached;
  double p_linear_vel, p_rot_vel;
  
  ros::Timer exploration_plan_generation_timer_;  // 勘探计划生成时间
  ros::Timer cmd_vel_generator_timer_;            // 定时解算速度（实时）
  ros::Timer sun_cmd_vel_generator_timer_;        // 定时解算速度（实时）
};

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SimpleExplorationController ec;

  ros::spin();

  return 0;
}
