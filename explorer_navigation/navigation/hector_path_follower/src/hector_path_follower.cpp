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

/*********************************************************************
* Based heavily on the pose_follower package
*********************************************************************/
#include <hector_path_follower/hector_path_follower.h>


namespace pose_follower {
  HectorPathFollower::HectorPathFollower(): tf_(NULL) {}

  void HectorPathFollower::initialize(tf::TransformListener* tf){
    tf_ = tf;
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    last_pose_time_ = ros::Time::now();
    ros::NodeHandle node_private("~/");

    node_private.param("k_trans", K_trans_, 2.0);
    node_private.param("k_rot", K_rot_, 2.0);

    node_private.param("tolerance_trans", tolerance_trans_, 0.1);//0.1
    node_private.param("tolerance_rot", tolerance_rot_, 0.3);//我的0.2 工控机0.4
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.2);
    //我的0.5 工控机0.1

    node_private.param("holonomic", holonomic_, true);//完整的　可积分的

    node_private.param("samples", samples_, 10);//样品；采样；例子

    node_private.param("max_vel_lin", max_vel_lin_, 1.0);//0.9  工控1.0
    node_private.param("max_vel_th", max_vel_th_, 2.0);  //1.4  工控2.0

    node_private.param("min_vel_lin", min_vel_lin_, 0.3);//0.1  工控0.3
    node_private.param("min_vel_th", min_vel_th_, 0.01);  //0.0  工控0.01
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.03);
    //0.0  工控0.03
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.02);
    //0.0  工控0.02

    node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
    node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

    node_private.param("robot_base_frame", p_robot_base_frame_, std::string("base_link"));
    node_private.param("global_frame", p_global_frame_, std::string("map"));

    //ros::NodeHandle node;
    //vel_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ROS_DEBUG("Initialized");
  }

  /*
  void HectorPathFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }
  */

  double HectorPathFollower::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
  {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }

  /*
  bool HectorPathFollower::stopped(){
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
  }
  */

  //计算速度的命令
  bool HectorPathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel,bool &isReached)
  {

    if (global_plan_.size() == 0){  //如果正常运行不会是零
        ROS_WARN("[In HectorPathFollower::computeVelocityCommands] global_plan_.size() == 0 ");
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.31;
        isReached = true;
        return false;
    } 

    //get the current pose of the robot in the fixed frame //在固定架的机器人的当前位置
    tf::Stamped<tf::Pose> robot_pose;
    if(!this->getRobotPose(robot_pose))
    {
      ROS_ERROR("Can't get robot pose");//如果tf转化超时，容易出现这个错误
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }
    
    /**
     * 前面保证得到了机器人的当前位置并且不是空的 正常运行不会进入上边的代码
     * we want to compute a velocity command based on our current waypoint
     * 们要计算一个基于我们当前位置速度命令
     */
    tf::Stamped<tf::Pose> target_pose;//目标位置
    tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);

    // /**********************辉巨*********************/
    // if (last_pose_time_ + ros::Duration(1.4) < ros::Time::now() )//在几秒内没有移动 然后发一个速度0
    // {
    //   geometry_msgs::Twist diff_tmp = diff2D(robot_pose, last_pose_);

    //   ROS_WARN("Sun:robot_pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
    //   ROS_WARN("Sun:last_pose_ %f %f ==> %f", last_pose_.getOrigin().x(), last_pose_.getOrigin().y(), tf::getYaw(last_pose_.getRotation()));
    //   ROS_WARN("Sun: diff_tmp x:%f y:%f ==> z:%f", diff_tmp.linear.x, diff_tmp.linear.y, diff_tmp.angular.z);
    //   ROS_WARN("tolerance_trans_:%f ", tolerance_trans_);
      
    //   last_pose_ = robot_pose;
    //   last_pose_time_ = ros::Time::now();
    //   if (fabs(diff_tmp.linear.x) <= tolerance_trans_ && fabs(diff_tmp.angular.z) <= tolerance_trans_)//0.1
    //   {
    //     ROS_ERROR("HectorPathFollower: find forbidden loop and we try to solve it");
    //     geometry_msgs::Twist empty_twist;
    //     cmd_vel = empty_twist;
    //     return true;
    //   }
    // }
    
    // /**************要删除还是要保留************/
    
    /**********************MaxChanger*********************/
    if (last_pose_time_ + ros::Duration(1.5) < ros::Time::now() )//在2秒内没有移动 
    {
      geometry_msgs::Twist diff_tmp = diff2D(robot_pose, last_pose_);
      last_pose_ = robot_pose;
      last_pose_time_ = ros::Time::now();
      if (fabs(diff_tmp.linear.x) <= tolerance_trans_ &&
          fabs(diff_tmp.linear.y) <= tolerance_trans_ &&
          fabs(diff_tmp.angular.z) <= tolerance_trans_ )//0.1
      {
        ROS_WARN("MaxChanger: robot_pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
        ROS_WARN("MaxChanger: last_pose_ %f %f ==> %f", last_pose_.getOrigin().x(), last_pose_.getOrigin().y(), tf::getYaw(last_pose_.getRotation()));
        ROS_WARN("MaxChanger: diff_tmp x:%f y:%f ==> z:%f", diff_tmp.linear.x, diff_tmp.linear.y, diff_tmp.angular.z);
        ROS_WARN("tolerance_trans_:%f ", tolerance_trans_);
        
        ROS_ERROR("MaxChanger: HectorPathFollower The robot don't move and we try to solve it\n");
        // cmd_vel.linear.x = -cmd_vel.linear.x;
        // cmd_vel.linear.y = -cmd_vel.linear.y;
        // cmd_vel.angular.z = -cmd_vel.angular.z; //既然不动 那么就是卡住了，速度往反方向走
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0.3; //既然不动 那么就是卡住了，速度往反方向走
        isReached = true;//然后假设到达目的地，重新规划路径
        return true;
      }
    }
    /**************要删除还是要保留************/


    ROS_WARN("global_plan_.size():%d next_goal_point: %d",global_plan_.size(),current_waypoint_);
    ROS_INFO("HectorPathFollower: Current Robot Pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
    ROS_INFO("HectorPathFollower: Target Robot Pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));
   
    //get the difference between the two poses
    geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);//得到目标位置和当前位置的区别


    ROS_INFO("HectorPathFollower: diff2D x:%f y:%f ==> z:%f", diff.linear.x, diff.linear.y, diff.angular.z);


    geometry_msgs::Twist limit_vel = limitTwist(diff);

    // ROS_INFO("HectorPathFollower: limit_vel x:%f y:%f ==> z:%f", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);

    geometry_msgs::Twist test_vel = limit_vel;
    bool legal_traj = true; 
    //collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, true);

    double scaling_factor = 1.0;
    double ds = scaling_factor / samples_;

    //let's make sure that the velocity command is legal... and if not, scale down 
    //让我们确保速度命令是合法的…如果没有，缩小规模
    if(!legal_traj){
      for(int i = 0; i < samples_; ++i)
      {
        test_vel.linear.x = limit_vel.linear.x * scaling_factor;
        test_vel.linear.y = limit_vel.linear.y * scaling_factor;
        test_vel.angular.z = limit_vel.angular.z * scaling_factor;
        test_vel = limitTwist(test_vel);
        //if(collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, false)){
          legal_traj = true;
          break;
        //}
        scaling_factor -= ds;
      }
    }

    if(!legal_traj)
    {
      ROS_ERROR("Not legal (%.2f, %.2f, %.2f)", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;//如果不合法 返回的是空的
      return false;
    }

    //if it is legal... we'll pass it on  如果是合法的…我们会把它传下去的
    //所以说传下去的应该是test_vel test_vel来自limit_vel limit_vel来自limitTwist(diff) diff = diff2D(target_pose, robot_pose)
    cmd_vel = test_vel;


    


      int ff_goal_size = 1;
      if(global_plan_.size() >= 150){
        ff_goal_size = global_plan_.size() - 20 ; 
      }else if(global_plan_.size() >= 80){
        ff_goal_size = global_plan_.size() * 0.9; 
      }else if(global_plan_.size() >= 50){
        ff_goal_size = global_plan_.size() * 0.8; 
      }else if(global_plan_.size() >= 10){
        ff_goal_size = global_plan_.size() - 5; 
      }else if (global_plan_.size() > 5){
        ff_goal_size = global_plan_.size() - 3; 
      }else{
        ff_goal_size = global_plan_.size() - 1;
      }

    if(ff_goal_size > 10){
      ff_goal_size = ( ff_goal_size / 5 ) * 5;
    }

    ROS_INFO("--->>>>if biggrer than %d I will stop ", ff_goal_size);



    bool in_goal_position = false;
    while(fabs(diff.linear.x) <= tolerance_trans_ &&
          fabs(diff.linear.y) <= tolerance_trans_ &&
          fabs(diff.angular.z) <= tolerance_rot_)
    {
      

      //if(current_waypoint_ < global_plan_.size() - 5)//-1   -7
      if(current_waypoint_ < ff_goal_size)
      {
        // if(ff_goal_size < 5){
        //   current_waypoint_ +=1;
        // }else if(ff_goal_size < 10){
        //   current_waypoint_ +=3;
        // }else{
          current_waypoint_ +=5;//+1  +7
        // }
        ROS_WARN("current_waypoint_: %d  ff_goal_size: %d" , current_waypoint_,ff_goal_size);
        tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
        diff = diff2D(target_pose, robot_pose);
      }
      else
      {
        ROS_WARN("Reached goal: %d", current_waypoint_);
        isReached = true;
        in_goal_position = true;
        break;
      }
    }

    //if we're not in the goal position, we need to update time
    //如果我们不在目标位置，我们需要更新时间
    if(!in_goal_position)
      goal_reached_time_ = ros::Time::now();
    else
    {
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
    }
    //check if we've reached our goal for long enough to succeed
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now())
    {
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
    }
    return true;
  }


  //设定计划
  bool HectorPathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    //path_follower_.setPlan(srv_setPlanexploration_plan.response.trajectory.poses);
    //geometry_msgs/PoseStamped[] poses 所以用一个容器（数组）来盛
    //global_plan是静态变量引入

    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    //去操作这个函数如果失败了 返回下面的错误 如果成功 完成了controller交给的任务 计划存到global_plan_
    //最后计划global_plan_被computeVelocityCommands计算速度的函数computeVelocityCommands调用
    if(!transformGlobalPlan(*tf_, global_plan, p_global_frame_, global_plan_)){//this->global_plan_ 
      //这个函数 global_plan是以静态常量传入函数的  global_plan_  是得到的局部路径规划
      //global_plan_ 是数组？？？
      
      ROS_ERROR("Could not transform the global plan to the frame of the controller");
      return false;
    }
    return true;
  }

  //目的地是否到达
  bool HectorPathFollower::isGoalReached()
  {
    /*
    //@TODO: Do something reasonable here
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now() && stopped()){
      return true;
    }
    */
    return false;
  }

  //pose1和pose2之间的不同    pose1: target_pose 目标位置, pose2: robot_pose 当前位置
  geometry_msgs::Twist HectorPathFollower::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
  {
    geometry_msgs::Twist res;

    // Transform inverse() const
    // { 
    //     Matrix3x3 inv = m_basis.transpose();
    //     return Transform(inv, inv * -m_origin);
    // } inverse的定义

    tf::Pose diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());

    if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
      return res;//holonomic_初始化的是true

    //in the case that we're not rotating to our goal position and we have a non-holonomic robot
    //we'll need to command a rotational velocity that will help us reach our desired heading
    
    //we want to compute a goal based on the heading difference between our pose and the target
    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
        pose2.getOrigin().x(), pose2.getOrigin().y(), tf::getYaw(pose2.getRotation()));

    //we'll also check if we can move more effectively backwards
    double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
        pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf::getYaw(pose2.getRotation()));

    //check if its faster to just back up
    if(fabs(neg_yaw_diff) < fabs(yaw_diff))
    {
      ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
      yaw_diff = neg_yaw_diff;
    }

    //compute the desired quaterion
    tf::Quaternion rot_diff = tf::createQuaternionFromYaw(yaw_diff);
    tf::Quaternion rot = pose2.getRotation() * rot_diff;
    tf::Pose new_pose = pose1;
    new_pose.setRotation(rot);

    diff = pose2.inverse() * new_pose;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());
    return res;
  }

  //限制速度
  geometry_msgs::Twist HectorPathFollower::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;//node_private.param("k_trans", K_trans_, 2.0);把k_trans值赋给K_trans_

    if(!holonomic_)//node_private.param("holonomic", holonomic_, true);
      res.linear.y = 0.0;
    else    
      res.linear.y *= K_trans_;// node_private.param("k_trans", K_trans_, 2.0);
    res.angular.z *= K_rot_;

    //make sure to bound things by our velocity limits 一定要结合我们的速度极限的东西 
    double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;

    double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
    /***************************/
    if (res.linear.x * res.linear.x + res.linear.y * res.linear.y < 1e-6)
    {
      lin_undershoot = 0.0;
    }
    /**************************/
    if (lin_overshoot > 1.0) 
    {
      res.linear.x /= lin_overshoot;
      res.linear.y /= lin_overshoot;
    }

    //we only want to enforce a minimum velocity if we're not rotating in place
    //我们只想执行一个最低速度，如果我们不旋转到位。 
    if(lin_undershoot > 1.0)
    {
      res.linear.x *= lin_undershoot;
      res.linear.y *= lin_undershoot;
    }

    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);

    //we want to check for whether or not we're desired to rotate in place 我们想检查一下我们是否需要轮换。 
    if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      res.linear.x = 0.0;
      res.linear.y = 0.0;
    }

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }

  bool HectorPathFollower::transformGlobalPlan( const tf::TransformListener& tf, 
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan )
      //transformed_plan是一个容器也只有这一个可以修改的量
      //最后计划global_plan_被computeVelocityCommands计算速度的函数computeVelocityCommands调用
  {
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try
    {
        if (!global_plan.size() > 0)
        {
          ROS_ERROR("Received plan with zero length");
          return false;
        }//程序正常的话不会进入这个if

        tf::StampedTransform transform;
        tf.lookupTransform(global_frame, ros::Time(), 
            plan_pose.header.frame_id, plan_pose.header.stamp, 
            plan_pose.header.frame_id, transform);

        tf::Stamped<tf::Pose> tf_pose;
        geometry_msgs::PoseStamped newer_pose;
        //now we'll transform until points are outside of our distance threshold
        //现在我们将转换直到点超出我们的距离阈值为止。
        for(unsigned int i = 0; i < global_plan.size(); ++i)
        {
          const geometry_msgs::PoseStamped& pose = global_plan[i];
          poseStampedMsgToTF(pose, tf_pose);
          tf_pose.setData(transform * tf_pose);
          tf_pose.stamp_ = transform.stamp_;
          tf_pose.frame_id_ = global_frame;
          poseStampedTFToMsg(tf_pose, newer_pose);

          transformed_plan.push_back(newer_pose);//transformed_plan是一个容器
        }
    }
    catch(tf::LookupException& ex) {//如果出错 才会进入catch 正常程序不会
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  //得到机器人的位置
  bool HectorPathFollower::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const 
  {

    global_pose.setIdentity();//设置身份

    tf::Stamped<tf::Pose> robot_pose;

    robot_pose.setIdentity();
    robot_pose.frame_id_ = p_robot_base_frame_;//将机器人当前位置的坐标系定位base_link

    robot_pose.stamp_ = ros::Time(0);
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    //get the global pose of the robot
    try{
      tf_->transformPose(p_global_frame_, robot_pose, global_pose);
      /**
       *p_global_frame = map
       *第一个参数是目标框架　也就是base_link
       *第二个参数应该是　父框架　到　子框架
       *void transformPose(const std::string& target_frame, const geometry_msgs::PoseStamped& stamped_in, geometry_msgs::PoseStamped& stamped_out) const;
       *进行转化之后　global_pose 应该有数据
       */
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    // check global_pose timeout

    /*
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
      ROS_WARN_THROTTLE(1.0, "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
          current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance_);
      return false;
    }
    */


    return true;
  }
};
