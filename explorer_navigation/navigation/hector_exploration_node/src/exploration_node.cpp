//======================================================================================
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
//======================================================================================

//1.作为服务器 接收来自客户端simple_exploration_controller探索路径的请求
//  调用explorationServiceCallback函数实现
//  explorationServiceCallback调用了planner里边的doExploration函数
//  最后的结果 既作为服务器回复客户端cotroller的请求 也发布到exploration_path话题(rviz可以选择订阅)
//


#include <ros/ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>

class SimpleExplorationPlanner
{
public:
  SimpleExplorationPlanner()
  {
      ros::NodeHandle nh;

      costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfl_);
      /**malloc了一块空间？
       * @brief Constructor for the wrapper.
       * Parameters:
       * name	The name for this costmap
       * tf	A reference to a TransformListener
       * 具体函数见READMEChanger.md
       */

      planner_ = new hector_exploration_planner::HectorExplorationPlanner();  //planner是Planner类型的指针

      planner_->initialize("hector_exploration_planner",costmap_2d_ros_);      //传入costmap_2d_ros_这个参数（传给其他参数），来进行初始化

      exploration_plan_service_server_ = nh.advertiseService("get_exploration_path", &
      
      SimpleExplorationPlanner::explorationServiceCallback, this);
      //进行到这一步的时候就已经开始调用callback函数
      ///这里是服务器 接收来自客户端simple_exploration_controller探索路径的请求

      exploration_plan_pub_ = nh.advertise<nav_msgs::Path>("exploration_path",2);
      //发布制定的计划到exploration_path话题 这个话题可以被rviz订阅
  }




  bool explorationServiceCallback(hector_nav_msgs::GetRobotTrajectory::Request  &req,
                                  hector_nav_msgs::GetRobotTrajectory::Response &res )
    {
      //看样子像是经过tf变换 既作为client的返回值 又作为信息发布到exploration_path
      ROS_WARN("Exploration Service Callback");

      tf::Stamped<tf::Pose> robot_pose_tf;
      
      costmap_2d_ros_->getRobotPose(robot_pose_tf); 
      /**
       * Get the pose of the robot in the global frame of the costmap. 
       * try
       * {
       *    tf_.transformPose(global_frame_, robot_pose, global_pose);
       * }
       * robor_pose_tf 得到的是 global_pose
       * 这样子进行了从机器人坐标系到世界坐标系的转换　得到了机器人对于全局的坐标系　
       * 存储在robot_pose_tf
       */
      
      geometry_msgs::PoseStamped pose;

      tf::poseStampedTFToMsg(robot_pose_tf , pose);
      /**
       * 将Stamped <Pose>转换成PoseStamped
       * convert Stamped<Pose> to PoseStamped msg
       * robot_pose_tf——>pose转化成消息类型
       */

      planner_->doExploration(pose , res.trajectory.poses);//调用函数得到res.   (得到exploration_path)
      /**
       *  全局路径的规划
       *  pose是机器人的现在的全局位置，以静态变量传入 也就是把机器人的当前位置传入
       *  (&start ，&plan） res.trajectory.poses引用传入 函数内部操作会存到该变量中
       */
     
      res.trajectory.header.frame_id = "map";
      res.trajectory.header.stamp = ros::Time::now();//时间戳

      /**
       * @brief getNumSubscribers() > 0 返回所有广播主题的订阅者总数。
       * 也就是说有人订阅的时候才发布这个话题，否则不发布这个话题
       * 这个publish写在了callback里边，也就是说把respond的东西不仅作为了回复，同时也发布到了话题
       * 既作为client的返回值 又作为信息发布到exploration_path
       */

      if (exploration_plan_pub_.getNumSubscribers() > 0)
      {
        exploration_plan_pub_.publish(res.trajectory);//这个发布了可以被rviz订阅 ("exploration_path",2);
      }

      return true;
    }



  protected:
    hector_exploration_planner::HectorExplorationPlanner* planner_;
    ros::ServiceServer exploration_plan_service_server_;    //这里一个服务器 接收来自客户端simple_exploration_controller探索路径的call
    ros::Publisher exploration_plan_pub_;                   //发布制定的计划到exploration_path话题  
    costmap_2d::Costmap2DROS* costmap_2d_ros_;
    tf::TransformListener tfl_;

};

int main(int argc, char **argv) {
  
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SimpleExplorationPlanner ep;

  ros::spin();

  return 0;

}
