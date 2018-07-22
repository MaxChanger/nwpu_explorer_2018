# Hector_navigation
Hector_navigation的代码从```roslaunch hector_exploration_node exploration_planner```开始
这个launch打开了```exploration_node```节点，调用了配置文件```costmap.yaml```

```
<launch>
  
  <node pkg="hector_exploration_node" type="exploration_planner_node" name="hector_exploration_node" output="screen">
    <param name="use_grid_map" value="true" />
    <rosparam file="$(find hector_exploration_node)/config/costmap.yaml" command="load" />
  </node>
  <!--node pkg="hector_exploration_controller" type="simple_exploration_controller" name="exploration_controller" output="screen"/-->
</launch>

```
costamp参数具体见文件

下面来看exploration_node.cpp
```
costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tfl_);
```
先是申请了一块空间用来存储获得的代价地图

```
exploration_plan_service_server_ = nh.advertiseService("get_exploration_path", &SimpleExplorationPlanner::explorationServiceCallback, this);
```
他是一个服务器，等待客户端```simple_exploration_controller```通过```“get_exploration_path”```的调用，然后返回探索出来的路径，探索路径的回调函数通过```explorationServiceCallback```函数实现
具体由下边这个调用实现，实现步骤在```hector_exploration_planner```里边实现
```
planner_->doExploration(pose , res.trajectory.poses);
```
然后得到的这个路径，这个路径既作为回调函数的返回值，也作为一个话题发布，加入订阅这个话题的节点的个数大于零，那么这个话题就会发布，比如rviz订阅这个话题。

综上所述，exploration_node的作用，是一个统筹的作用，调用exploration_planner的函数来实现得到路径，接收来自客户端simple_exploration_controller探索路径的call，把速度发给底盘接收的话题。


---

# hector_exploration_controller


```
exploration_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");
```
首先这里定义了一个client，通过"get_exploration_path"向exploration_node索要路径的回复
```
path_follower_.initialize(&tfl_);
```
这个是对Path_follower类的初始化

```
exploration_plan_generation_timer_ = nh.createTimer(ros::Duration(0.3), &SimpleExplorationController::timerPlanExploration, this, false );
//探索路径计划生成时间
cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(0.1), &SimpleExplorationController::timerCmdVelGeneration, this, false );
//定时解算速度（实时）
```

> ros::Timer ros::NodeHandle::createTimer(ros::Duration period, <callback>, bool oneshot = false);  
函数说明  
period ，这是调用定时器回调函数时间间隔。例如，ros::Duration(0.1)，即每十分之一秒执行一次  
回调函数，可以是函数，类方法，函数对象。  
oneshot ，表示是否只执行一次，如果已经执行过，还可以通过stop()、setPeriod(ros::Duration)和start()来规划再执行一次。
回调函数用法

```
void timerPlanExploration(const ros::TimerEvent& e)
```
函数请求```(call)exploration_node```给他一个探索路径的回复（并且广播了出去）然后```node```调用```planner```里边的```doExploration```返回的答案存在了```srv_exploration_plan```里面。  
然后又调用了```path_follower```里边的setplan函数,setplan函数用到了刚才返回的结果作为参数,然后setPlan又调用follower里边的```transformGlobalPlan```函数结果存在了```global_plan_```中,最后```global_plan_```经过下面```computeVelocityCommands```计算速度的时候调用了

```
void timerCmdVelGeneration(const ros::TimerEvent& e)

vel_pub_ = nh.advertise<geometry_msgs::Twist>("/explorer_drive_controller/cmd_vel", 10);
```
该函数调用path_follower里边的computeVelocityCommands（计算下一步的速度）computeVelocityCommands用了global_plan_的数据,最后生成要发布的速度存在了twist里边,通过"/explorer_drive_controller/cmd_vel"话题发送给底盘


---
# hector_path_follower
```
bool HectorPathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
```
先通过自身的另一个函数```this->getRobotPose(robot_pose)```得到机器人的位置，最后经过一番计算，处理过的速度还是返回在cmd_vel中

```
 tf::poseStampedMsgToTF(global_plan_[current_waypoint_], target_pose);
 //将global_plan_里边的数据转化掉目标位置target_pose
 geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);  
 //得到目标位置和当前位置的区别
 geometry_msgs::Twist limit_vel = limitTwist(diff);
 //将速度限制住
```



```
bool HectorPathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
```
传入的global_plan是simple_exploration_controller请求exploration_node，然后调用exploration_planner里边的函数最后得到的路径

>if(!transformGlobalPlan(*tf_, global_plan, p_global_frame_, global_plan_))

通过调用```transformGlobalPlan```函数，把call到的全局的global_plan转化为局部的global_plan_


```
 bool HectorPathFollower::transformGlobalPlan( const tf::TransformListener& tf, 
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan )
```
>transformed_plan.push_back(newer_pose);  
  
不断的向这个队列中压入新的值，在这个位置调用传入的是global_plan_


```
bool HectorPathFollower::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
```
>tf_->transformPose(p_global_frame_, robot_pose, global_pose);  

>void transformPose(const std::string& target_frame, const geometry_msgs::PoseStamped& stamped_in, geometry_msgs::PoseStamped& stamped_out) const

---
# hector_exploration_planner
这个是一个不断被调用的类，并没有直接被运行起来，而是在不断的被node在调用其函数


```
ExplorationPlanner.cfg
```
这个文件是用来进行动态调参的一个配置文件，同时也可以在里边设置一些初始值

```
rosrun rqt_reconfigure rqt_reconfigure  //打开动态调参的rqt节点
```

先放上几个参数的值
```
namespace costmap_2d
{
    static const unsigned char NO_INFORMATION = 255;//没有信息
    static const unsigned char LETHAL_OBSTACLE = 254;//致命的障碍
    static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;//内切 膨胀障碍
    static const unsigned char FREE_SPACE = 0;
}

#define STRAIGHT_COST 120   //直接成本 new120_old100
#define DIAGONAL_COST 156   //对角线成本 new156_old141


模式选择
enum LastMode
  {
    FRONTIER_EXPLORE,
    INNER_EXPLORE
  }last_mode_;
```
>this->costmap_ros_ = costmap_ros_in; //导入代价地图   
>this->setupMapData(); 


```
visualization_pub_ = private_nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);//visualization_marker  可视化标记

observation_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("observation_pose", 1, true);//观察姿势

goal_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, true);

///发布目标位置等可视化信息  在rviz中可以看到

path_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");
//设置了一个服务器，向hector_trajectory索要数据
```

```
void HectorExplorationPlanner::dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level)
```
这个函数是用作给动态调参用的，对于算法部分没有什么影响


```
bool HectorExplorationPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &original_goal, std::vector<geometry_msgs::PoseStamped> &plan)
```
// do exploration? (not used anymore? -> call doExploration()) 这个函数好像被放弃了


```
bool HectorExplorationPlanner::doExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan)
```
给定一个起点，在已知和未知的空间之间找到一个边界，并产生一个去那里的计划  
在这个函数中，调用了很多函数，```buildobstacle_trans_array_(p_use_inflated_obs_)```建立一个数组，这个数组用来存储代价值，```p_explore_close_to_path_```这个参数决定了是否按照接近原来的路径的方式进行下一步的道路探索，个人理解这个参数如果是true的话，机器人规划的路径将会很依赖于原来的路径，经常出现按原路返回的情况
````
//这是根据这个参数调用的两个不同的函数，传入参数相同，只是函数处理方式不同
if（p_explore_close_to_path_   is  true ）
	frontiers_found = findFrontiersCloseToPath(goals);
else
	frontiers_found = findFrontiers(goals);


if(frontiers_found)//如果找到边界了
  {
    last_mode_ = FRONTIER_EXPLORE;
  } 
  else //如果没有找到边界 进行inner探索（内部探索）
  {
    return doInnerExploration(start,plan);//具体参见后边该函数解释
  }
````
```
if(!buildexploration_trans_array_(start,goals,true)) //建立了探索路径数组
if(!getTrajectory(start,goals,plan))//得到了到目标点的路径
``` 
```
bool HectorExplorationPlanner::doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan)
```
当系统没有发现机器人姿态可达的任何边界时，切换到内部探索模式，从hector_trajectory_server请求机器人轨迹。 然后尝试去远离机器人轨迹的地图上的位置（例如，去未被访问的地方） 转入内部探索不应该在启动时发生  
个人理解这个函数的发生应该在全部地图都建立好（所有边界都封锁好了）之后，很遗憾到目前位置还没有遇见过这种情况的发生