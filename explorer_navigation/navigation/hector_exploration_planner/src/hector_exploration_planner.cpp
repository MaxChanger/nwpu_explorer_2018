
/**
 * 该软件包提供了一个规划器，可以为探索未知环境生成目标和相关路径。 该包不提供ROS节点，而是可以在ROS节点内部使用的库
 * 可以调用该库中的函数 
 * 该软件包提供了一个规划器，可以为探索未知环境生成目标和相关路径。 该包不提供ROS节点，而是可以在ROS节点内部使用的库。
 * namespace costmap_2d
 * {
 *  static const unsigned char NO_INFORMATION = 255;//没有信息
 *  static const unsigned char LETHAL_OBSTACLE = 254;//致命的障碍
 *  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;//内切 膨胀障碍
 *  static const unsigned char FREE_SPACE = 0;
 *  }
 * 
 */

#include <hector_exploration_planner/hector_exploration_planner.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <Eigen/Geometry>
#include <hector_exploration_planner/ExplorationPlannerConfig.h>

#define STRAIGHT_COST 100   //120   //直接成本 白工控机写的120 --100
#define DIAGONAL_COST 170   //156   //对角线成本 白工控机写的156 --141

//#define STRAIGHT_COST 3
//#define DIAGONAL_COST 4

using namespace hector_exploration_planner;

HectorExplorationPlanner::HectorExplorationPlanner()
: costmap_ros_(0)
, costmap_(0)
, initialized_(false)
, map_width_(0)
, map_height_(0)
, num_map_cells_(0)
{}

HectorExplorationPlanner::~HectorExplorationPlanner()
{
  this->deleteMapData();
}//析构函数 最后调用清除数据内存空间




HectorExplorationPlanner::HectorExplorationPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in) :
costmap_ros_(NULL), initialized_(false) 
{
  HectorExplorationPlanner::initialize(name, costmap_ros_in);
  //把exploration_node里边的costmap_2d_ros_传入给costmap_ros_in
  //然后costmap_ros_in后来又给了costmap_ros_（这个量是在hector_exploration_planner里边的一个量）
}




void HectorExplorationPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros_in)
{
  //初始化函数
  last_mode_ = FRONTIER_EXPLORE;
  // unknown: 255, obstacle 254, inflated: 253, free: 0

  /**
   *模式选择
   enum LastMode
   {
        FRONTIER_EXPLORE,
        INNER_EXPLORE
   }last_mode_;
   */

  
  if(initialized_)//如果已经初始化 会有提示 //为什么initialized函数调用2次
  {
    ROS_ERROR("[hector_exploration_planner] HectorExplorationPlanner is already initialized_! Please check why initialize() got called twice.");
    return;//返回 不会执行下面步骤
  }

  ROS_INFO("[hector_exploration_planner] Initializing Hector Exploration Planner"); //正常情况下会提示正在初始化

  // initialize costmaps    初始化代价地图
  this->costmap_ros_ = costmap_ros_in;
  this->setupMapData();

  // initialize parameters  初始化参数
  ros::NodeHandle private_nh_("~/" + name);
  ros::NodeHandle nh;

  visualization_pub_ = private_nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);//visualization_marker  可视化标记

  observation_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("observation_pose", 1, true);//观察姿势

  goal_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, true);

  
  ///发布目标位置等可视化信息  在rviz中可以看到

  ignore_all_ = false;
  
  dyn_rec_server_.reset(new dynamic_reconfigure::Server<hector_exploration_planner::ExplorationPlannerConfig>(ros::NodeHandle("~/hector_exploration_planner")));//动态调参

  dyn_rec_server_->setCallback(boost::bind(&HectorExplorationPlanner::dynRecParamCallback, this, _1, _2));

  path_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");
  //这是一个客户端 向hector_trajecrory发送请求 call    1000多行 if (path_service_client_.call(srv_path))

  ROS_INFO("[hector_exploration_planner] Parameter set. security_const: %f, min_obstacle_dist: %d, plan_in_unknown: %d, use_inflated_obstacle: %d, p_goal_angle_penalty_:%d , min_frontier_size: %d, p_dist_for_goal_reached_: %f, same_frontier: %f", p_alpha_, p_min_obstacle_dist_, p_plan_in_unknown_, p_use_inflated_obs_, p_goal_angle_penalty_, p_min_frontier_size_,p_dist_for_goal_reached_,p_same_frontier_dist_);
  //p_min_obstacle_dist_ = p_min_obstacle_dist_ * STRAIGHT_COST;

  this->name = name;
  this->initialized_ = true;
  this->previous_goal_ = -1;

  vis_.reset(new ExplorationTransformVis("exploration_transform"));
  close_path_vis_.reset(new ExplorationTransformVis("close_path_exploration_transform"));
  inner_vis_.reset(new ExplorationTransformVis("inner_exploration_transform"));
  obstacle_vis_.reset(new ExplorationTransformVis("obstacle_transform"));
}

//动态调参函数
void HectorExplorationPlanner::dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level)
{
  p_plan_in_unknown_ = config.plan_in_unknown;
  p_explore_close_to_path_ = config.explore_close_to_path;//默认值是ture
  p_use_inflated_obs_ = config.use_inflated_obstacles;
  p_goal_angle_penalty_ = config.goal_angle_penalty;
  p_alpha_ = config.security_constant;
  p_dist_for_goal_reached_ = config.dist_for_goal_reached;
  p_same_frontier_dist_ = config.same_frontier_distance;//0.25 (0~10)
  p_min_frontier_size_ = config.min_frontier_size;
  p_min_obstacle_dist_ = config.min_obstacle_dist * STRAIGHT_COST;
  p_obstacle_cutoff_dist_ = config.obstacle_cutoff_distance;

  p_use_observation_pose_calculation_ = config.use_observation_pose_calculation;
  p_observation_pose_desired_dist_ = config.observation_pose_desired_dist;
  double angle_rad = config.observation_pose_allowed_angle * (M_PI / 180.0);
  p_cos_of_allowed_observation_pose_angle_ = cos(angle_rad);
  p_close_to_path_target_distance_ = config.close_to_path_target_distance;
}




bool HectorExplorationPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &original_goal, std::vector<geometry_msgs::PoseStamped> &plan)
{

  this->setupMapData();

  // do exploration? (not used anymore? -> call doExploration())

  if ((original_goal.pose.orientation.w == 0.0) && (original_goal.pose.orientation.x == 0.0) &&
  (original_goal.pose.orientation.y == 0.0) && (original_goal.pose.orientation.z == 0.0))
  {
      ROS_ERROR("Trying to plan with invalid quaternion, this shouldn't be done anymore, but we'll start exploration for now.");
      // 尝试用无效四元数进行规划，这不应该再做了，但是我们现在就开始探索。
      return doExploration(start,plan);
  }


  // planning
  ROS_INFO("[hector_exploration_planner] planning: starting to make a plan to given goal point");

  // setup maps and goals
  resetMaps();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  // create obstacle tranform
  // buildobstacle_trans_array_(p_use_inflated_obs_);

  goal_pose_pub_.publish(original_goal);//通过"goal_pose"话题发布 原始的 位置

  geometry_msgs::PoseStamped adjusted_goal;

  if (p_use_observation_pose_calculation_)//利用观测姿态计算 cfg给的是true
  {
    ROS_INFO("Using observation pose calc.");
    if (!this->getObservationPose(original_goal, p_observation_pose_desired_dist_, adjusted_goal))
    {
      ROS_ERROR("getObservationPose returned false, no area around target point available to drive to!");
      return false;
    }
  }
  else
  {
    ROS_INFO("Not using observation pose calc.");
    this->buildobstacle_trans_array_(true);
    adjusted_goal = original_goal;
  }

  observation_pose_pub_.publish(adjusted_goal);//通过"observation_pose"发布话题 调整的位置

  // plan to given goal
  goals.push_back(adjusted_goal);

  // make plan
  if(!buildexploration_trans_array_(start,goals,true))//建立探测阵列 
  {
    return false;
  }

  if(!getTrajectory(start,goals,plan)){
    return false;
  }

  // save and add last point
  plan.push_back(adjusted_goal);
  unsigned int mx,my;
  costmap_->worldToMap(adjusted_goal.pose.position.x,adjusted_goal.pose.position.y,mx,my);
  previous_goal_ = costmap_->getIndex(mx,my);

  if ((original_goal.pose.orientation.w == 0.0) && (original_goal.pose.orientation.x == 0.0) &&
  (original_goal.pose.orientation.y == 0.0) && (original_goal.pose.orientation.z == 0.0)){
      geometry_msgs::PoseStamped second_last_pose;
      geometry_msgs::PoseStamped last_pose;
      second_last_pose = plan[plan.size()-2];
      last_pose = plan[plan.size()-1];
      last_pose.pose.orientation = second_last_pose.pose.orientation;
      plan[plan.size()-1] = last_pose;
  }

  ROS_INFO("[hector_exploration_planner] planning: plan has been found! plan.size: %u ", (unsigned int)plan.size());
  return true;
}




/**
 * 给定一个起点，在已知和未知的空间之间找到一个边界，并产生一个去那里的计划
 */
bool HectorExplorationPlanner::doExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan)
{

    this->setupMapData();   // setup maps and goals setup maps and goals

    resetMaps();
    clearFrontiers();       //清理边界 把边界数组置为0
    plan.clear();           //函数clear()删除储存在vector中的所有元素. 

    std::vector<geometry_msgs::PoseStamped> goals;//目标
    
    /**
     * create obstacle transform 创造障碍变换
     * use inflated obstacle 是否使用膨胀半径当作参数传入
     * 经过这个函数的处理 最小代价移动数组已经完成 
     */
    buildobstacle_trans_array_(p_use_inflated_obs_);
    //建立一个数组，这个数组用来存储代价值  使不使用膨胀半径只是其一个参数
  
    bool frontiers_found = false;//假设没有发现边缘

    if (p_explore_close_to_path_)//默认值是ture 是否依据现有路径进行下一步的路径规划
    {
      frontiers_found = findFrontiersCloseToPath(goals);//存到了goals里边
      //找到靠近路径的边界
      //以走过的路径为基准点 进行下一步的探索

      if (!frontiers_found){
        ROS_WARN("Close Exploration desired, but no frontiers found. Falling back to normal exploration!");
        //想要进行接近与原来路径的探索 然而失败之后就返回正常的探索模式 
        //需要进行严密勘探，但没有发现边界。落回正常的探索！ 
        frontiers_found = findFrontiers(goals);
      }

    }else{
      frontiers_found = findFrontiers(goals);//这种是没有依靠原路径的搜索
    }

    // search for frontiers
    if(frontiers_found)//如果找到边界了
    {
      last_mode_ = FRONTIER_EXPLORE;
      ROS_INFO("[hector_exploration_planner] exploration: found %u frontiers!", (unsigned int)goals.size());
      //输出找到多少个边界点？？？？？？？？？？？？？？？？
    } 
    else //如果没有找到边界 进行inner探索（内部探索）
    {
      ROS_INFO("[hector_exploration_planner] exploration: no frontiers have been found! starting inner-exploration");
      return doInnerExploration(start,plan);
    }
    
  
    // make plan
    if(!buildexploration_trans_array_(start,goals,true)) // 生产了可到达的点goals
    {
      return false;
    }

    ROS_ERROR("[Sun_exploration_planner] current position :\n x:%f \t y:%f \t z:%f \n" ,
              start.pose.position.x ,start.pose.position.y ,start.pose.position.z);

    // for(unsigned int i = 0; i < goals.size(); ++i)// 输出所有找到的goals 但是路径只是去其中一个goal
    // {
    //       ROS_ERROR("[Sun_exploration_planner] goals[%d] :\n x:%f \t y:%f \t z:%f " ,
    //         i, goals[i].pose.position.x ,goals[i].pose.position.y ,goals[i].pose.position.z);
    // }
    /**
     * 正常步骤(其实调用函数是个主要的 看其返回值) 
     * get到了 返回1 !1=0 不进入 也是在这个函数里边 plan得到数据
     * start是静态变量 goals只是传入 最后得到plan
     * 与上边一样同样失败之后开启内部探索模式
     */
    if(!getTrajectory(start,goals,plan)){
      
      ROS_INFO("[hector_exploration_planner] exploration: could nopath_service_client_t plan to frontier, starting inner-exploration");
      //没有路径服务客户端T计划前沿，开始内部探索
      return doInnerExploration(start,plan);
    }


    // 限定在第一种探索模式下 plan 的长度
    if( last_mode_ == FRONTIER_EXPLORE && plan.size() > 50){
      while(plan.size() > 50){
        plan.pop_back();
      }
    }






    // update previous goal 更新以前的目标
    if(!plan.empty())//加入现在已经有了移动计划 计划不是空 那么进入下面语句
    {
      geometry_msgs::PoseStamped thisgoal = plan.back();
      unsigned int mx,my;
      costmap_->worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
      previous_goal_ = costmap_->getIndex(mx,my);
    }



    // for(unsigned int i = 0; i < plan.size(); ++i) // 循环输出所有路径，其实看起来没有什么意义
    // {
    //       ROS_ERROR("[Sun_exploration_planner] plan_route_point[%d] :\n x:%f \t y:%f \t z:%f " ,
    //         i, plan[i].pose.position.x ,plan[i].pose.position.y ,plan[i].pose.position.z);
    // }
    int i = plan.size()-1;
    ROS_ERROR("[Sun_exploration_planner] plan_route_point[%d] :\n x:%f \t y:%f \t z:%f " ,
            i, plan[i].pose.position.x ,plan[i].pose.position.y ,plan[i].pose.position.z);
    
    //这条信息被输出 输出的是路线的长度（也就是有几个栅格点）
    ROS_INFO("[hector_exploration_planner] exploration: plan to a frontier has been found! plansize: %u", (unsigned int)plan.size());
    return true;
}




/**
 * When the system does not find any frontiers reachable from the pose of the robot, it switches to inner exploration mode, which requests the robot trajectory from hector_trajectory_server. It then tries to go to locations in the map that are far away from the trajectory of the robot (e.g. go to places not visited yet). Switching into inner exploration really shouldn´t happen at startup though, so something is going wrong. 
 * 当系统没有发现机器人姿态可达的任何边界时，切换到内部探索模式，从hector_trajectory_server请求机器人轨迹。 然后尝试去远离机器人轨迹的地图上的位置（例如，去未被访问的地方）。 转入内部探索真的不应该在启动时发生，所以出了问题。
 * 
 * You can also take a look at the hector_exploration_planner. It has a "inner exploration" mode that will keep visiting places in the map even after the complete environment has been explored. 
 * 你也可以看看hector_exploration_planner。 它有一个“内在探索”的模式，即使在探索完整的环境之后，仍然会继续在地图上访问。
 * 
 * 如果地图中没有未知的边界，则可以使用此边界。
 * 机器人将通过一个服务检索到它迄今为止走过的路径，并尝试到距离这条路很远的地方。
 * Parameters 参数
 * start	The start point 启动起点
 * plan	The plan to explore into unknown space 计划探索未知的空间
 */
bool HectorExplorationPlanner::doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan)
{
  ROS_INFO("[hector_exploration_planner] inner-exploration: starting exploration");
  // setup maps and goals
  resetMaps();
  clearFrontiers();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  // create obstacle tranform
  buildobstacle_trans_array_(p_use_inflated_obs_);

  // If we have been in inner explore before, check if we have reached the previous inner explore goal
  if (last_mode_ == INNER_EXPLORE)
  {
    tf::Stamped<tf::Pose> robotPose;
    if(!costmap_ros_->getRobotPose(robotPose))
    {
      ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
    }

    unsigned int xm, ym;
    costmap_->indexToCells(previous_goal_, xm, ym);

    double xw, yw;
    costmap_->mapToWorld(xm, ym, xw, yw);

    double dx = xw - robotPose.getOrigin().getX();
    double dy = yw - robotPose.getOrigin().getY();

    //If we have not reached the previous goal, try planning and moving toward it.
    //如果我们还没有达到过上一个目标，就试着去计划并朝着这个目标迈进。
    //If planning fails, we just continue below this block and try to find another inner frontier  如果计划失败了，我们只能继续在这个区块的下面，试图找到另一个内在的边界
    if ( (dx*dx + dy*dy) > 0.5*0.5)
    {

      geometry_msgs::PoseStamped robotPoseMsg;
      tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

      geometry_msgs::PoseStamped goalMsg;
      goalMsg.pose.position.x = xw;
      goalMsg.pose.position.y = yw;
      goalMsg.pose.orientation.w = 1.0;

      if(makePlan(robotPoseMsg, goalMsg, plan))
      {
        //Successfully generated plan to (previous) inner explore goal
        ROS_INFO("[hector_exploration_planner] inner-exploration: Planning to previous inner frontier");
        last_mode_ = INNER_EXPLORE;
        return true;
      }
    }

  }

  // search for frontiers
  if(findInnerFrontier(goals))
  {
    ROS_INFO("[hector_exploration_planner] inner-exploration: found %u inner-frontiers!", (unsigned int)goals.size());
  } 
  else 
  {
    ROS_WARN("[hector_exploration_planner] inner-exploration: no inner-frontiers have been found! exploration failed!");
    return false;
  }

  // make plan
  if(!buildexploration_trans_array_(start,goals,false))
  {
    ROS_WARN("[hector_exploration_planner] inner-exploration: Creating exploration transform failed!");
    return false;
  }
  if(!getTrajectory(start,goals,plan))
  {
    ROS_WARN("[hector_exploration_planner] inner-exploration: get plan failed in doInnerExploration");
    return false;
  }

  // cutoff last points of plan due to sbpl error when planning close to walls

  int plansize = plan.size() - 5;
  if(plansize > 0 )
  {
    plan.resize(plansize);///////////////////resize 
  }

  // update previous goal
  if(!plan.empty())
  {
    const geometry_msgs::PoseStamped& thisgoal = plan.back();
    unsigned int mx,my;
    costmap_->worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
    previous_goal_ = costmap_->getIndex(mx,my);
    last_mode_ = INNER_EXPLORE;
  }

  ROS_INFO("[hector_exploration_planner] inner-exploration: plan to an inner-frontier has been found! plansize: %u", (unsigned int)plan.size());
  return true;
}




bool HectorExplorationPlanner::getObservationPose(const geometry_msgs::PoseStamped& observation_pose, const double desired_distance, geometry_msgs::PoseStamped& new_observation_pose)
{
  // We call this from inside the planner, so map data setup and reset already happened 我们从计划者内部调用它，所以地图数据的设置和重置已经发生了。
  //this->setupMapData();
  //resetMaps();

  if (!p_use_observation_pose_calculation_){
    ROS_WARN("getObservationPose was called although use_observation_pose_calculation param is set to false. Returning original pose!");
    new_observation_pose = observation_pose;
    this->buildobstacle_trans_array_(true);
    return true;
  }

  unsigned int mxs,mys;
  costmap_->worldToMap(observation_pose.pose.position.x, observation_pose.pose.position.y, mxs, mys);

  double pose_yaw = tf::getYaw(observation_pose.pose.orientation);

  Eigen::Vector2f obs_pose_dir_vec (cos(pose_yaw), sin(pose_yaw));

  this->buildobstacle_trans_array_(true);

  int searchSize = 2.0 / costmap_->getResolution();

  int min_x = mxs - searchSize/2;
  int min_y = mys - searchSize/2;

  if (min_x < 0){
    min_x = 0;
  }

  if (min_y < 0){
    min_y = 0;
  }

  int max_x = mxs + searchSize/2;
  int max_y = mys + searchSize/2;

  if (max_x > static_cast<int>(costmap_->getSizeInCellsX())){
    max_x = static_cast<int>(costmap_->getSizeInCellsX()-1);
  }

  if (max_y > static_cast<int>(costmap_->getSizeInCellsY())){
    max_y = static_cast<int>(costmap_->getSizeInCellsY()-1);
  }

  int closest_x = -1;
  int closest_y = -1;

  unsigned int closest_sqr_dist = UINT_MAX;

  bool no_information = true;

  for (int x = min_x; x < max_x; ++x){
    for (int y = min_y; y < max_y; ++y){

      unsigned int point = costmap_->getIndex(x,y);

      unsigned int obstacle_trans_val = obstacle_trans_array_[point];

      if ( (obstacle_trans_val != UINT_MAX) && (obstacle_trans_val != 0) && (occupancy_grid_array_[point] != costmap_2d::NO_INFORMATION)){

        no_information = false;

        int diff_x = x - (int)mxs;
        int diff_y = y - (int)mys;

        unsigned int sqr_dist = diff_x*diff_x + diff_y*diff_y;

        //std::cout << "diff: " << diff_x << " , " << diff_y << " sqr_dist: " << sqr_dist << " pos: " << x << " , " << y << " closest sqr dist: " << closest_sqr_dist << " obstrans " << obstacle_trans_array_[costmap_->getIndex(x,y)] << "\n";

        if (sqr_dist < closest_sqr_dist){

          Eigen::Vector2f curr_dir_vec(static_cast<float>(diff_x), static_cast<float>(diff_y));
          curr_dir_vec.normalize();

          if (curr_dir_vec.dot(obs_pose_dir_vec) <  p_cos_of_allowed_observation_pose_angle_){

            closest_x = (unsigned int)x;
            closest_y = (unsigned int)y;
            closest_sqr_dist = sqr_dist;
          }
        }
      }
    }
  }

  if (no_information){
    new_observation_pose.pose = observation_pose.pose;
    new_observation_pose.pose.position.z = 0.0;
    ROS_INFO("Observation pose unchanged as no information available around goal area");
    return true;
  }

  //std::cout << "start: " << mxs << " , " << mys << " min: " << min_x << " , " << min_y << " max: " <<  max_x << " , " << max_y << "\n";
  //std::cout << "pos: " << closest_x << " , " << closest_y << "\n";

  // Found valid pose if both coords are larger than -1
  if ((closest_x > -1) && (closest_y > -1)){

    Eigen::Vector2d closest_point_world;
    costmap_->mapToWorld(closest_x, closest_y, closest_point_world.x(),  closest_point_world.y());

    Eigen::Vector2d original_goal_pose(observation_pose.pose.position.x, observation_pose.pose.position.y);

    //geometry_msgs::PoseStamped pose;
    new_observation_pose.header.frame_id = "map";
    new_observation_pose.header.stamp = observation_pose.header.stamp;

    Eigen::Vector2d dir_vec(original_goal_pose - closest_point_world);

    double distance = dir_vec.norm();

    //If we get back the original observation pose (or another one very close to it), return that
    if (distance < (costmap_->getResolution() * 1.5)){
      new_observation_pose.pose = observation_pose.pose;
      new_observation_pose.pose.position.z = 0.0;
      ROS_INFO("Observation pose unchanged");
    }else{

      if (desired_distance < distance){
        new_observation_pose.pose.position.x = closest_point_world.x();
        new_observation_pose.pose.position.y = closest_point_world.y();
        new_observation_pose.pose.position.z = 0.0;
      }else{

        double test_distance = distance;

        Eigen::Vector2d last_valid_pos(closest_point_world);

        do{
          test_distance += 0.1;

          double distance_factor = test_distance / distance;

          Eigen::Vector2d new_pos(original_goal_pose - dir_vec*distance_factor);

          unsigned int x, y;
          costmap_->worldToMap(new_pos[0], new_pos[1], x, y);
          unsigned int idx = costmap_->getIndex(x,y);

          if(!this->isFree(idx)){
            break;
          }

          last_valid_pos = new_pos;

        }while (test_distance < desired_distance);

        new_observation_pose.pose.position.x = last_valid_pos.x();
        new_observation_pose.pose.position.y = last_valid_pos.y();
        new_observation_pose.pose.position.z = 0.0;
      }

      double angle = std::atan2(dir_vec.y(), dir_vec.x());

      new_observation_pose.pose.orientation.w = cos(angle*0.5);
      new_observation_pose.pose.orientation.x = 0.0;
      new_observation_pose.pose.orientation.y = 0.0;
      new_observation_pose.pose.orientation.z = sin(angle*0.5);
      ROS_INFO("Observation pose moved away from wall");
    }

    return true;
  }else{
    // If closest vals are still -1, we didn't find a position
    ROS_ERROR("Couldn't find observation pose for given point.");
    return false;
  }
}



//做选择的探索 没有用到
bool HectorExplorationPlanner::doAlternativeExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &oldplan)
{
  ROS_INFO("[hector_exploration_planner] alternative exploration: starting alternative exploration");

  // setup maps and goals
  resetMaps();
  plan.clear();

  std::vector<geometry_msgs::PoseStamped> goals;

  std::vector<geometry_msgs::PoseStamped> old_frontier;
  old_frontier.push_back(oldplan.back());

  // create obstacle tranform
  buildobstacle_trans_array_(p_use_inflated_obs_);

  // search for frontiers
  if(findFrontiers(goals,old_frontier)){
    ROS_INFO("[hector_exploration_planner] alternative exploration: found %u frontiers!", (unsigned int) goals.size());
  } else {
    ROS_WARN("[hector_exploration_planner] alternative exploration: no frontiers have been found!");
    return false;
  }

  // make plan
  if(!buildexploration_trans_array_(start,goals,true)){
    return false;
  }
  if(!getTrajectory(start,goals,plan)){
    return false;
  }

  const geometry_msgs::PoseStamped& this_goal = plan.back();
  unsigned int mx,my;
  costmap_->worldToMap(this_goal.pose.position.x,this_goal.pose.position.y,mx,my);
  previous_goal_ = costmap_->getIndex(mx,my);

  ROS_INFO("[hector_exploration_planner] alternative exploration: plan to a frontier has been found! plansize: %u ", (unsigned int)plan.size());
  return true;
}



float HectorExplorationPlanner::angleDifferenceWall(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
  // setup start positions
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle){
    both_angle = start_angle - goal_angle;
  } else {
    both_angle = goal_angle - start_angle;
  }

  return both_angle;
}

// 貌似也没用到
bool HectorExplorationPlanner::exploreWalls(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){

  //@TODO: Properly set this parameters
  int startExploreWall = 1;

  ROS_DEBUG("[hector_exploration_planner] wall-follow: exploreWalls");
  unsigned int mx,my;
  if(!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)){
    ROS_WARN("[hector_exploration_planner] wall-follow: The start coordinates are outside the costmap!");
    return false;
  }
  int currentPoint=costmap_->getIndex(mx, my);
  int nextPoint;
  int oldDirection = -1;
  int k=0;
  int loop=0;

  while(k<50){
    int adjacentPoints [8];
    getAdjacentPoints(currentPoint, adjacentPoints);
    int dirPoints [3];

    unsigned int minDelta = UINT_MAX;
    unsigned int maxDelta = 0;
    unsigned int thisDelta;
    float minAngle=3.1415; //Rad -> 180°

    geometry_msgs::PoseStamped trajPoint;
    unsigned int gx,gy;

    if(oldDirection==-1){
      // find robot orientation
      for ( int i=0; i<8; i++){
        costmap_->indexToCells((unsigned int)adjacentPoints[i],gx,gy);
        double wx,wy;
        costmap_->mapToWorld(gx,gy,wx,wy);
        std::string global_frame = costmap_ros_->getGlobalFrameID();
        trajPoint.header.frame_id = global_frame;
        trajPoint.pose.position.x = wx;
        trajPoint.pose.position.y = wy;
        trajPoint.pose.position.z = 0.0;
        float yaw = angleDifferenceWall(start, trajPoint);
        if(yaw < minAngle){
          minAngle=yaw;
          oldDirection=i;
        }
      }
    }

    //search possible orientation

    if (oldDirection == 0){
      dirPoints[0]=oldDirection+4; //right-forward
      dirPoints[1]=oldDirection;   //forward
      dirPoints[2]=oldDirection+7; //left-forward
    }
    else if (oldDirection < 3){
      dirPoints[0]=oldDirection+4;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection+3;
    }
    else if (oldDirection == 3){
      dirPoints[0]=oldDirection+4;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection+3;
    }
    else if (oldDirection == 4){
      dirPoints[0]=oldDirection-3;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection-4;
    }
    else if (oldDirection < 7){
      dirPoints[0]=oldDirection-3;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection-4;
    }
    else if (oldDirection == 7){
      dirPoints[0]=oldDirection-7;
      dirPoints[1]=oldDirection;
      dirPoints[2]=oldDirection-4;
    }

    // decide LHR or RHR
    if(startExploreWall == -1){
      if(obstacle_trans_array_[adjacentPoints[dirPoints[0]]] <= obstacle_trans_array_[adjacentPoints[dirPoints[2]]]){
        startExploreWall = 0;
        ROS_INFO("[hector_exploration_planner] wall-follow: RHR");//mirror inverted??
      }
      else {
        startExploreWall = 1;
        ROS_INFO("[hector_exploration_planner] wall-follow: LHR");//mirror inverted??
      }
    }



    //switch left and right, LHR
    if(startExploreWall == 1){
      int temp=dirPoints[0];
      dirPoints[0]=dirPoints[2];
      dirPoints[2]=temp;
    }


    // find next point
    int t=0;
    for(int i=0; i<3; i++){
      thisDelta = obstacle_trans_array_[adjacentPoints[dirPoints[i]]];

      if (thisDelta > 3000 || loop > 7) // point is unknown or robot drive loop
      {
        int plansize = plan.size() - 4;
        if(plansize > 0 ){
          plan.resize(plansize);
        }
        ROS_DEBUG("[hector_exploration_planner] wall-follow: END: exploreWalls. Plansize %d", (int)plan.size());
        return !plan.empty();
      }

      if(thisDelta >= (unsigned int) p_min_obstacle_dist_){
        if(obstacle_trans_array_[currentPoint] >= (unsigned int) p_min_obstacle_dist_){
          if(abs(thisDelta - p_min_obstacle_dist_) < minDelta){
            minDelta = abs(thisDelta - p_min_obstacle_dist_);
            nextPoint = adjacentPoints[dirPoints[i]];
            oldDirection = dirPoints[i];
          }
        }
        if(obstacle_trans_array_[currentPoint] < (unsigned int) p_min_obstacle_dist_){
          if(thisDelta > maxDelta){
            maxDelta = thisDelta;
            nextPoint = adjacentPoints[dirPoints[i]];
            oldDirection = dirPoints[i];
          }
        }
      }
      else {
        if(thisDelta < obstacle_trans_array_[currentPoint]){
          t++;
        }
        if(thisDelta > maxDelta){
          maxDelta = thisDelta;
          nextPoint = adjacentPoints[dirPoints[i]];
          oldDirection = dirPoints[i];

        }
      }
    }

    if(t==3 && abs(obstacle_trans_array_[adjacentPoints[dirPoints[0]]] - obstacle_trans_array_[adjacentPoints[dirPoints[1]]]) < STRAIGHT_COST
    && abs(obstacle_trans_array_[adjacentPoints[dirPoints[0]]] - obstacle_trans_array_[adjacentPoints[dirPoints[2]]]) < STRAIGHT_COST
    && abs(obstacle_trans_array_[adjacentPoints[dirPoints[1]]] - obstacle_trans_array_[adjacentPoints[dirPoints[2]]]) < STRAIGHT_COST){
      nextPoint=adjacentPoints[dirPoints[2]];
      oldDirection=dirPoints[2];
    }

    if(oldDirection==dirPoints[2])
      loop++;
    else
      loop=0;

    // add point
    unsigned int sx,sy;
    costmap_->indexToCells((unsigned int)currentPoint,sx,sy);
    costmap_->indexToCells((unsigned int)nextPoint,gx,gy);
    double wx,wy;
    costmap_->mapToWorld(sx,sy,wx,wy);
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    trajPoint.header.frame_id = global_frame;
    trajPoint.pose.position.x = wx;
    trajPoint.pose.position.y = wy;
    trajPoint.pose.position.z = 0.0;
    // assign orientation
    int dx = gx-sx;
    int dy = gy-sy;
    double yaw_path = std::atan2(dy,dx);
    trajPoint.pose.orientation.x = 0.0;
    trajPoint.pose.orientation.y = 0.0;
    trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
    trajPoint.pose.orientation.w = cos(yaw_path*0.5f);
    plan.push_back(trajPoint);

    currentPoint=nextPoint;
    k++;
  }
  ROS_DEBUG("[hector_exploration_planner] wall-follow: END: exploreWalls. Plansize %d", (int)plan.size());
  return !plan.empty();
}


void HectorExplorationPlanner::setupMapData()
{

  /**
   * @brief ？？？？？？？？？？？？
   * 如果满足条件 就得到代价地图
   * 否则 清空代价地图数据 得到一个代价地图的copy
   */

  #ifdef COSTMAP_2D_LAYERED_COSTMAP_H_
    costmap_ = costmap_ros_->getCostmap();
  #else
    if (costmap_) delete costmap_;
    costmap_ = new costmap_2d::Costmap2D;
    costmap_ros_->getCostmapCopy(*costmap_);
  #endif

  //Below code can be used to guarantee start pose is cleared. Somewhat risky.
  //下面的代码可以用来保证开始姿势被清除。有些冒险的。

  //@TODO: Make available through dynamic reconfigure 通过动态重新配置提供
  /*
  std::vector<geometry_msgs::Point> points;
  costmap_ros_->getOrientedFootprint(points);

  bool filled = costmap_->setConvexPolygonCost(points, costmap_2d::FREE_SPACE);

  //std::vector<geometry_msgs::Point> points = costmap_ros_->getRobotFootprint();
  for (size_t i = 0; i < points.size(); ++i)
    std::cout << points[i];
  if (filled)
    ROS_INFO("Set costmap to free");
  else
    ROS_INFO("Failed to set costmap free");
  */
  
  /**
   * getSizeInCellsX()
   * Accessor for the x size of the costmap in cells.
   * Returns:
   * The x size of the costmap
   * 如果现在地图的宽不等于代价地图的x 或者现在的高不等于代价地图的y
   * this->map_width_  this->map_height_ 被初始化为0  所以这个函数第一次应该是肯定进入的
   */

  if ((this->map_width_ != costmap_->getSizeInCellsX()) || (this->map_height_ != costmap_->getSizeInCellsY()))
  {
    map_width_ = costmap_->getSizeInCellsX();   //得到地图的宽
    map_height_ = costmap_->getSizeInCellsY();  //得到地图的高
    
    num_map_cells_ = map_width_ * map_height_;  //栅格（数组）的数量

    /**
     * @initialize                初始化
     * exploration_trans_array_   路径探索数组
     * obstacle_trans_array_      障碍物数组
     * goalMap                    全局地图
     * frontier_map_array_        边界地图数组
     */
    exploration_trans_array_.reset(new unsigned int[num_map_cells_]);
    obstacle_trans_array_.reset(new unsigned int[num_map_cells_]);
    is_goal_array_.reset(new bool[num_map_cells_]);
    ///
    in_queue_.reset(new bool[num_map_cells_]); //区别
    ///
    frontier_map_array_.reset(new int[num_map_cells_]);

    clearFrontiers();//frontier_map_array_ 全置为 0
    resetMaps();
    /** resetMaps():作用
     * std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
     * 把探索的路径的数组填成最大值
     * std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
     * 把障碍物的路径的数组填成最大值
     * std::fill_n(is_goal_array_.get(), num_map_cells_, false);
     * 把是不是目标的数组全填成 不是
     */
  }

  occupancy_grid_array_ = costmap_->getCharMap();
  //将返回一个不可变的指针，作为成本映射的底层unsigned char数组。
  //getCharMap() ====  return costmap_;

}

void HectorExplorationPlanner::deleteMapData()//析构函数调用此函数 清空内存空间
{
  exploration_trans_array_.reset();
  obstacle_trans_array_.reset();
  is_goal_array_.reset();
  frontier_map_array_.reset();
  ///
  in_queue_.reset();
  ///
}

//取用goals
bool HectorExplorationPlanner::buildexploration_trans_array_(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, bool useAnglePenalty, bool use_cell_danger)
{

  //这个函数 对exploration_trans_array_[]数组进行计算填充
  //exploration_trans_array_[] 这个数组的值 是代价值
  //之后的getTrajectory函数根据这个数组 找最短路径 根据每一步的代价值最小的原则
  //建立探测阵列 建立探索路径的数组 使用角度的处罚? 使用细胞危险

  ROS_DEBUG("[hector_exploration_planner] buildexploration_trans_array_");

  // reset exploration transform
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);//先都填上最大值
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);
  ///
  std::fill_n(in_queue_.get(), num_map_cells_, false);
  ///
  std::queue<int> myqueue;

  size_t num_free_goals = 0;

  // initialize goals
  for(unsigned int i = 0; i < goals.size(); ++i)
  {
    // setup goal positions 设置目标位置
    unsigned int mx,my;

    if(!costmap_->worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my))//将世界坐标系转化为地图坐标系
    {
      //ROS_WARN("[hector_exploration_planner] The goal coordinates are outside the costmap!");
      continue;
    }

    int goal_point = costmap_->getIndex(mx,my);//行列转化为第几个 索引

    // Ignore free goal for the moment, check after iterating over all goals if there is not valid one at all 暂时忽略自由目标，如果没有有效目标，检查所有目标后迭代。

    if(!isFree(goal_point))//体现在地图上就是是不是Free 白色的无障碍的地方
    {
      continue;//如果不是 看下一个
    }
    else
    {
      ++num_free_goals;//如果是的话 空白目标的个数++
    }

    unsigned int init_cost = 0;
    if(false)
    {
      init_cost = angleDanger(angleDifference(start,goals[i])) * getDistanceWeight(start,goals[i]);
    }//这句话肯定是不会执行的,更多是注释的作用

    exploration_trans_array_[goal_point] = init_cost;

    // do not punish previous frontiers (oscillation)
    if(false && isValid(previous_goal_))
    {
      if(isSameFrontier(goal_point, previous_goal_))
      {
        ROS_DEBUG("[hector_exploration_planner] same frontier: init with 0");
        exploration_trans_array_[goal_point] = 0;
      }
    }

    ROS_DEBUG("[hector_exploration_planner] Goal init cost: %d, point: %d", exploration_trans_array_[goal_point], goal_point);
    is_goal_array_[goal_point] = true; //判断是不是目标数组 置为1
    myqueue.push(goal_point);//将目标的点压入队列
    ///
    in_queue_[goal_point] = true;
    ///
  }//end for

  if (num_free_goals == 0)//如果没有找到一个目标点的话
  {
    ROS_WARN("[hector_exploration_planner] All goal coordinates for exploration transform invalid (occupied or out of bounds), aborting.");
    return false;
  }

  // exploration transform algorithm 探索变换算法
  if (use_cell_danger) //不同的传入的这个值还不一样
  {
    while(myqueue.size())
    {
      int point = myqueue.front();//取队首元素
      myqueue.pop();//弹出队首元素
      ///
      in_queue_[point] = false;
      ///

      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];//straightPoints[0][1][2][3]分别是上下左右四个方向的坐标
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];//这两个数组一共代表了八邻域
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      // 计算所有相邻单元格的最小探测值
      // if ( 当前点的值 + 两点之间的成本 + cellDanger < 邻域点的值 ）
      //     邻域点的值 = 当前点的值 + 两点之间的成本 + cellDanger
      for (int i = 0; i < 4; ++i) 
      {
        if (isFree(straightPoints[i]))
        {
          unsigned int neighbor_cost = minimum + STRAIGHT_COST + cellDanger(straightPoints[i]);//相邻的八邻域点成本

          if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) 
          {
            exploration_trans_array_[straightPoints[i]] = neighbor_cost;
            ///
            if (false == in_queue_[straightPoints[i]] || ignore_all_)
	          {
              myqueue.push(straightPoints[i]);
            	in_queue_[straightPoints[i]] = true;
            }
            ///
          }
        }

        if (isFree(diagonalPoints[i])) 
        {
          unsigned int neighbor_cost = minimum + DIAGONAL_COST + cellDanger(diagonalPoints[i]);

          if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) 
          {
            exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
            if (false == in_queue_[diagonalPoints[i]] || ignore_all_)
	          {
              myqueue.push(diagonalPoints[i]);
            	in_queue_[diagonalPoints[i]] = true;
            }
            
          }
        }
      
      }//end for
    }//end while
  }//end if(use_cell_danger) 
  else
  {
    while(myqueue.size())
    {
      int point = myqueue.front();
      myqueue.pop();
      ///
      in_queue_[point] = false;
      ///
      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      for (int i = 0; i < 4; ++i) {
        if (isFree(straightPoints[i])) {
          unsigned int neighbor_cost = minimum + STRAIGHT_COST;

          if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) {
            exploration_trans_array_[straightPoints[i]] = neighbor_cost;
              ///
            	if (false == in_queue_[straightPoints[i]] || ignore_all_)
              {
                myqueue.push(straightPoints[i]);
                in_queue_[straightPoints[i]] = true;
              }
              ///
          }
        }

        if (isFree(diagonalPoints[i])) {
          unsigned int neighbor_cost = minimum + DIAGONAL_COST;

          if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) {
            exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
            	///
              if (false == in_queue_[diagonalPoints[i]] || ignore_all_)
              {
                myqueue.push(diagonalPoints[i]);
                in_queue_[diagonalPoints[i]] = true;
              }
              ///
          }
        }
      }
    }
  }

  ROS_DEBUG("[hector_exploration_planner] END: buildexploration_trans_array_");

  vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());//点云？
  return true;
}

bool HectorExplorationPlanner::buildobstacle_trans_array_(bool use_inflated_obstacles)
{
  //建立跨阵列的障碍
  
  ROS_DEBUG("[hector_exploration_planner] buildobstacle_trans_array_");
 
  std::queue<int> myqueue;
  ///
  std::fill_n(in_queue_.get(), num_map_cells_, false);
  ///

  /**
   * init obstacles                     初始化障碍物
   * FREE_SPACE = 0
   * LETHAL_OBSTACLE = 254              致命的障碍
   * INSCRIBED_INFLATED_OBSTACLE = 253  内切的膨胀的障碍
   * NO_INFORMATION = 255
   * 
   */
  for(unsigned int i=0; i < num_map_cells_; ++i)
  { //把致命的点和膨胀半径的点加入到要循环的队列
    if(occupancy_grid_array_[i] == costmap_2d::LETHAL_OBSTACLE)//254
    {
      myqueue.push(i);
      ///
      in_queue_[i] = true;
      ///

      obstacle_trans_array_[i] = 0;
    }
    else if(use_inflated_obstacles)//如果使用了膨胀半径
    {
      if(occupancy_grid_array_[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)//253
      {
        myqueue.push(i);
        ///
        in_queue_[i] = true;
        ///
        obstacle_trans_array_[i] = 0;
      }
    }
  }
  //定义了障碍的临界值
  unsigned int obstacle_cutoff_value = static_cast<unsigned int>((p_obstacle_cutoff_dist_ / costmap_->getResolution()) * STRAIGHT_COST + 0.5);

  ROS_ERROR("resolution: %f", costmap_->getResolution());
  // obstacle transform algorithm   障碍变换算法
  while(myqueue.size())//当我的队列不为空的时候
  {
    int point = myqueue.front();//取队列顶部存放于 point
    myqueue.pop();//弹出队顶元素
    ///
    in_queue_[point] = false;
    ///

    unsigned int minimum = obstacle_trans_array_[point];//取当前的障碍变换数组
    
    if (minimum > obstacle_cutoff_value) 
        continue;
      /**
       * 如果当前点的最小障碍变换数组要比障碍临界值搭大的话继续下一个
       * 直到找到一当前障碍变换数组的值比障碍的临界值要小的值
       * 这样才进行下边的部分
       */

    int straightPoints[4];//straight Points直点
    getStraightPoints(point,straightPoints);
    
    int diagonalPoints[4];//diagonal Points对角点 八邻域
    getDiagonalPoints(point,diagonalPoints);

    /**
     * check all 8 directions   检查八个方向 
     * 如果这个点是有效的 并且他的 当前障碍变换数组的值大于最小值+直接成本
     * 则替换
     * 有点像dijkstra
     * if（A当前值 + AB之间成本 < B当前值）
     *    B的当前值 = A当前值 + AB之间成本 
     */
    for(int i = 0; i < 4; ++i)
    {
      if (isValid(straightPoints[i]) && (obstacle_trans_array_[straightPoints[i]] > minimum + STRAIGHT_COST))
      {
        obstacle_trans_array_[straightPoints[i]] = minimum + STRAIGHT_COST;
        ///
        if (false == in_queue_[straightPoints[i]] || ignore_all_)
        {
          myqueue.push(straightPoints[i]);
          in_queue_[straightPoints[i]] = true;
        }
        ///
      }

      if (isValid(diagonalPoints[i]) && (obstacle_trans_array_[diagonalPoints[i]] > minimum + DIAGONAL_COST)) 
      {
        obstacle_trans_array_[diagonalPoints[i]] = minimum + DIAGONAL_COST;
        ///
        if (false == in_queue_[diagonalPoints[i]] || ignore_all_)
        {
          myqueue.push(diagonalPoints[i]);
          in_queue_[diagonalPoints[i]] = true;
        }
        ///
      }
    }
  }

  ROS_DEBUG("[hector_exploration_planner] END: buildobstacle_trans_array_");

  obstacle_vis_->publishVisOnDemand(*costmap_, obstacle_trans_array_.get());
  return true;
}

bool HectorExplorationPlanner::getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan)
{ //这个函数 ×可能是通过梯度下降算法× 得到起始点到目标点的最快路径 然后保存到了plan里边
  //doExploration调用 得到start到goals的plan
  //start是静态变量 goals只是传入 最后得到plan

  ROS_DEBUG("[hector_exploration_planner] getTrajectory");

  unsigned int mx,my;// setup start positions 设置起始位置

  if(!costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mx,my))
  {
    ROS_WARN("[hector_exploration_planner] The start coordinates are outside the costmap!");//开始坐标在costmap之外
    return false;
  }

  int currentPoint = costmap_->getIndex(mx,my);//当前位置  行列转化为第几个
  int nextPoint = currentPoint;//下一个位置 先用当前位置初始化

  geometry_msgs::PoseStamped trajPoint;//轨迹点
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  trajPoint.header.frame_id = global_frame;

  if (is_goal_array_[currentPoint])//当前点是否是目标点 判断是否到了目标点
  {
    ROS_INFO("Already at goal point position. No pose vector generated.");
    //已经在目标点位置。 没有姿势矢量生成
    return true;
  }

  // 这个寻找plan的路径 是从start开始，然后一直循环，找当前点八邻域代价值最小的点，然后不停地+1，直到遇到一个点不满足
  // while(!is_goal_array_[currentPoint]) ————— 也就是 当到达的点是goal的时候，plan的不断push结束 - 找到exploration_plan


  while(!is_goal_array_[currentPoint])//当当前点不是障碍物点的时候 直到碰到一个goal点 while循环结束
  {//这是个while循环 没有到达目标点就一直生成trajPoint(轨迹点)数据 然后压入plan队列
    int thisDelta;
    int adjacentPoints[8];//八邻域
    getAdjacentPoints(currentPoint,adjacentPoints);

    int maxDelta = 0;

    // 选择的nextPonit是当前点的八邻域里边 ××（thisDelta）最大的一个点
    for(int i = 0; i < 8; ++i){

      if(isFree(adjacentPoints[i])){

        thisDelta = exploration_trans_array_[currentPoint] - exploration_trans_array_[adjacentPoints[i]];
        if(thisDelta > maxDelta)
        {
          maxDelta = thisDelta;
          nextPoint = adjacentPoints[i];
        }
      }
    }

    // This happens when there is no valid exploration transform data at the start point for example 
    // 例如，在起始点没有有效的勘探转换数据时会发生这种情况
    if(maxDelta == 0)
    {
      ROS_WARN("[hector_exploration_planner] In getTrajectory: No path to the goal could be found by following gradient!");
      //没有路径的目标可以通过下面的梯度找到！
      return false;
    }

    // make trajectory point 制作轨迹点
    unsigned int sx,sy,gx,gy;
    costmap_->indexToCells((unsigned int)currentPoint,sx,sy);
    costmap_->indexToCells((unsigned int)nextPoint,gx,gy);
    double wx,wy;
    costmap_->mapToWorld(sx,sy,wx,wy);

    trajPoint.pose.position.x = wx;
    trajPoint.pose.position.y = wy;
    trajPoint.pose.position.z = 0.0;

    // assign orientation
    int dx = gx-sx;
    int dy = gy-sy;
    double yaw_path = std::atan2(dy,dx);
    trajPoint.pose.orientation.x = 0.0;
    trajPoint.pose.orientation.y = 0.0;
    trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
    trajPoint.pose.orientation.w = cos(yaw_path*0.5f);

    plan.push_back(trajPoint);//把生成的可以进行的轨迹点 压入计划中 之后进行下一次
    currentPoint = nextPoint;//把下一个点当做当前点进行迭代
    maxDelta = 0;
  }//while循环结束 没有到达目的地就一直生成trajPoint数据 然后压入plan


  ROS_DEBUG("[hector_exploration_planner] END: getTrajectory. Plansize %u", (unsigned int)plan.size());
  return !plan.empty();
}



bool HectorExplorationPlanner::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers)
{
  std::vector<geometry_msgs::PoseStamped> empty_vec;
  return findFrontiers(frontiers,empty_vec);
}

/*
 * searches the occupancy grid for frontier cells 
 * and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 * 搜索边界单元的占用网格，并将其合并到每个边界的一个目标点。
 * 返回的边界是在世界坐标。
 */
bool HectorExplorationPlanner::findFrontiersCloseToPath(std::vector<geometry_msgs::PoseStamped> &frontiers)
{
  clearFrontiers();
  frontiers.clear();//frontiers数组清空

  // get the trajectory as seeds for the exploration transform 
  // 得到的轨迹为探索转化种子 
  hector_nav_msgs::GetRobotTrajectory srv_path;

  if (path_service_client_.call(srv_path))
  {
    /**
     * path_service_client_ = 
     * nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory"); 
     * 发送请求 得到路径 如果找到了 进入下面的步骤
     */
    std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);
    // 这个vextor用来存储call得到的以前的路径

    // We push poses of the travelled trajectory to the goals vector 
    // for building the exploration transform
    // 我们把行进轨迹的姿态推到目标矢量上，以建立勘探变换。把路径的数据放到一个vector里边
    std::vector<geometry_msgs::PoseStamped> goals;

    size_t size = traj_vector.size();

    ROS_INFO("[hector_exploration_planner] Size of trajectory vector for close exploration %u", (unsigned int)size);

    if(size > 0)//如果路径的vector不是空的
    {
      geometry_msgs::PoseStamped lastPose = traj_vector[size-1];
      //lastPose 是走过路径的最后一个点
      //把这最后一个点也当做是目标的第一个点
      goals.push_back(lastPose);

      if (size > 1)
      {
        for(int i = static_cast<int>(size-2); i >= 0; --i)
        { //从倒数第二个点开始 逐一递减遍历
          const geometry_msgs::PoseStamped& pose = traj_vector[i];
          unsigned int x,y;
          costmap_->worldToMap(pose.pose.position.x,pose.pose.position.y,x,y);
          /**
           * 1 wx 	The x world coordinate
           * 2 wy 	The y world coordinate
           * 3 mx 	Will be set to the associated map x coordinate
           * 4 my 	Will be set to the associated map y coordinate
           * 从世界坐标pose.pose.position.x 转换到地图坐标 x。
           */

          unsigned int m_point = costmap_->getIndex(x,y);
          /**
           * 给定两个地图坐标…计算关联索引。 将二维的行列 转化为 一维的第几个
           * inline unsigned int getIndex(unsigned int mx, unsigned int my) const
           * {
           *     return  my * size_x_ + mx;
           * }
          **/
          double dx = lastPose.pose.position.x - pose.pose.position.x;
          double dy = lastPose.pose.position.y - pose.pose.position.y;

          if((dx*dx) + (dy*dy) > (0.25*0.25))
          {
            goals.push_back(pose);
            lastPose = pose;
          }
        
        }//end for

        ROS_INFO("[hector_exploration_planner] pushed %u goals (trajectory) for close to robot frontier search", (unsigned int)goals.size());
        //接近机器人边界搜索

        // make exploration transform
        tf::Stamped<tf::Pose> robotPose;
        if(!costmap_ros_->getRobotPose(robotPose))//得到的是机器人全局坐标
        {
          //Get the pose of the robot in the global frame of the costmap.
          //如果没有得到机器人的位置 发出错误提示
          ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
        }

        geometry_msgs::PoseStamped robotPoseMsg;
        tf::poseStampedTFToMsg(robotPose, robotPoseMsg);//转化为消息类型

        if (!buildexploration_trans_array_(robotPoseMsg, goals, false, false))
        { //这个函数填上了数组 exploration_trans_array_[]
          ROS_WARN("[hector_exploration_planner]: Creating exploration transform array in find inner frontier failed, aborting.");
          //创建勘探变换数组 寻找内部边界失败 中止。
          return false;
        }

        close_path_vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());
        //按需发布？publish Vis On Demand 点云？

        unsigned int explore_threshold = static_cast<unsigned int> (static_cast<double>(STRAIGHT_COST) * (1.0/costmap_->getResolution()) * p_close_to_path_target_distance_);//explore 探索的阈值

        //std::vector<geometry_msgs::PoseStamped> close_frontiers;

        for(unsigned int i = 0; i < num_map_cells_; ++i)
        {
          unsigned int current_val = exploration_trans_array_[i];
          if(current_val < UINT_MAX)//#define UINT_MAX 4294967295U
          {
            if (current_val >= explore_threshold){ //&& current_val <= explore_threshold+ DIAGONAL_COST){
              geometry_msgs::PoseStamped finalFrontier;//最后的边界
              double wx,wy;
              unsigned int mx,my;
              costmap_->indexToCells(i, mx, my);//把一维数组的第几个转化为行列
              costmap_->mapToWorld(mx,my,wx,wy);//把map坐标系的转化到世界坐标系

              std::string global_frame = costmap_ros_->getGlobalFrameID();
              finalFrontier.header.frame_id = global_frame;
              finalFrontier.pose.position.x = wx;
              finalFrontier.pose.position.y = wy;
              finalFrontier.pose.position.z = 0.0;

              double yaw = getYawToUnknown(costmap_->getIndex(mx,my));//偏航到未知

              //if(frontier_is_valid){

              finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

              frontiers.push_back(finalFrontier);//逐一压入要返回的边界
            }
          }
        }

        return frontiers.size() > 0;

      }
    }
  }



  // list of all frontiers in the occupancy grid
  std::vector<int> allFrontiers;

  // check for all cells in the occupancy grid whether or not they are frontier cells
  for(unsigned int i = 0; i < num_map_cells_; ++i)
  {
    if(isFrontier(i))
    {
      allFrontiers.push_back(i);
    }
  }

  for(unsigned int i = 0; i < allFrontiers.size(); ++i)
  {
    if(!isFrontierReached(allFrontiers[i]))
    {
      geometry_msgs::PoseStamped finalFrontier;
      double wx,wy;
      unsigned int mx,my;
      costmap_->indexToCells(allFrontiers[i], mx, my);
      costmap_->mapToWorld(mx,my,wx,wy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wx;
      finalFrontier.pose.position.y = wy;
      finalFrontier.pose.position.z = 0.0;

      double yaw = getYawToUnknown(costmap_->getIndex(mx,my));

      //if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      frontiers.push_back(finalFrontier);
    }
    //}
  }

  return (frontiers.size() > 0);
}

/*
 * searches the occupancy grid for frontier cells and merges them into one target point per frontier.
 * The returned frontiers are in world coordinates.
 * 搜索边界单元的占用网格，并将其合并到每个边界的一个目标点。
 * 返回的边界是在世界坐标。
 */
bool HectorExplorationPlanner::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers)
{

  // get latest costmap 获取最新的成本地图
  clearFrontiers();

  // list of all frontiers in the occupancy grid 列出栅格地图中所有边界的
  std::vector<int> allFrontiers;

  // check for all cells in the occupancy grid whether or not they are frontier cells   检查占用网格中的所有单元格，不管它们是否是边界单元
  for(unsigned int i = 0; i < num_map_cells_; ++i)
  {
    if(isFrontier(i))
    {
      allFrontiers.push_back(i);//如果这个点是边界 先压入vector
    }
  }

  for(unsigned int i = 0; i < allFrontiers.size(); ++i)//遍历所有的边界点
  {
    if(!isFrontierReached(allFrontiers[i]))
    {
      geometry_msgs::PoseStamped finalFrontier;
      double wx,wy;
      unsigned int mx,my;

      costmap_->indexToCells(allFrontiers[i], mx, my);
      costmap_->mapToWorld(mx,my,wx,wy);

      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wx;
      finalFrontier.pose.position.y = wy;
      finalFrontier.pose.position.z = 0.0;

      double yaw = getYawToUnknown(costmap_->getIndex(mx,my));

      //if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      frontiers.push_back(finalFrontier);
      
      //}
    }
  }

  return (frontiers.size() > 0);

  //@TODO: Review and possibly remove unused code below审查并可能删除下面的未使用的代码

  // value of the next blob 下一个blob的值
  int nextBlobValue = 1;
  std::list<int> usedBlobs;

  for(unsigned int i = 0; i < allFrontiers.size(); ++i){

    // get all adjacent blobs to the current frontier point
    int currentPoint = allFrontiers[i];
    int adjacentPoints[8];
    getAdjacentPoints(currentPoint,adjacentPoints);

    std::list<int> blobs;

    for(int j = 0; j < 8; j++){
      if(isValid(adjacentPoints[j]) && (frontier_map_array_[adjacentPoints[j]] > 0)){
        blobs.push_back(frontier_map_array_[adjacentPoints[j]]);
      }
    }
    blobs.unique();

    if(blobs.empty()){
      // create new blob
      frontier_map_array_[currentPoint] = nextBlobValue;
      usedBlobs.push_back(nextBlobValue);
      nextBlobValue++;
    } else {
      // merge all found blobs
      int blobMergeVal = 0;

      for(std::list<int>::iterator adjBlob = blobs.begin(); adjBlob != blobs.end(); ++adjBlob){
        if(adjBlob == blobs.begin()){
          blobMergeVal = *adjBlob;
          frontier_map_array_[currentPoint] = blobMergeVal;
        } else {

          for(unsigned int k = 0; k < allFrontiers.size(); k++){
            if(frontier_map_array_[allFrontiers[k]] == *adjBlob){
              usedBlobs.remove(*adjBlob);
              frontier_map_array_[allFrontiers[k]] = blobMergeVal;
            }
          }
        }
      }
    }
  }

  int id = 1;

  bool visualization_requested = (visualization_pub_.getNumSubscribers() > 0);

  // summarize every blob into a single point (maximum obstacle_trans_array_ value)
  for(std::list<int>::iterator currentBlob = usedBlobs.begin(); currentBlob != usedBlobs.end(); ++currentBlob){
    int current_frontier_size = 0;
    int max_obs_idx = 0;

    for(unsigned int i = 0; i < allFrontiers.size(); ++i){
      int point = allFrontiers[i];

      if(frontier_map_array_[point] == *currentBlob){
        current_frontier_size++;
        if(obstacle_trans_array_[point] > obstacle_trans_array_[allFrontiers[max_obs_idx]]){
          max_obs_idx = i;
        }
      }
    }

    if(current_frontier_size < p_min_frontier_size_){
      continue;
    }

    int frontier_point = allFrontiers[max_obs_idx];
    unsigned int x,y;
    costmap_->indexToCells(frontier_point,x,y);

    // check if frontier is valid (not to close to robot and not in noFrontiers vector
    bool frontier_is_valid = true;

    if(isFrontierReached(frontier_point)){
      frontier_is_valid = false;
    }

    for(size_t i = 0; i < noFrontiers.size(); ++i){
      const geometry_msgs::PoseStamped& noFrontier = noFrontiers[i];
      unsigned int mx,my;
      costmap_->worldToMap(noFrontier.pose.position.x,noFrontier.pose.position.y,mx,my);
      int no_frontier_point = costmap_->getIndex(x,y);
      if(isSameFrontier(frontier_point,no_frontier_point)){
        frontier_is_valid = false;
      }
    }

    geometry_msgs::PoseStamped finalFrontier;
    double wx,wy;
    costmap_->mapToWorld(x,y,wx,wy);
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    finalFrontier.header.frame_id = global_frame;
    finalFrontier.pose.position.x = wx;
    finalFrontier.pose.position.y = wy;
    finalFrontier.pose.position.z = 0.0;

    double yaw = getYawToUnknown(costmap_->getIndex(x,y));

    if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      frontiers.push_back(finalFrontier);
    }

    // visualization (export to method?)
    if(visualization_requested){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "hector_exploration_planner";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.position.z = 0.0;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;

      if(frontier_is_valid){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
      }else{
        marker.color.r = 1.0;
        marker.color.g = 0.0;
      }

      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(5,0);
      visualization_pub_.publish(marker);
    }

  }
  return !frontiers.empty();
}

bool HectorExplorationPlanner::findInnerFrontier(std::vector<geometry_msgs::PoseStamped> &innerFrontier)
{
  clearFrontiers();

  // get the trajectory as seeds for the exploration transform
  hector_nav_msgs::GetRobotTrajectory srv_path;
  if (path_service_client_.call(srv_path))
  {
    ROS_WARN("findInnerFrontier____(path_service_client_.call(srv_path)");
    std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

    // We push poses of the travelled trajectory to the goals vector for building the exploration transform
    std::vector<geometry_msgs::PoseStamped> goals;

    size_t size = traj_vector.size();
    ROS_DEBUG("[hector_exploration_planner] size of trajectory vector %u", (unsigned int)size);

    if(size > 0)
    {
      geometry_msgs::PoseStamped lastPose = traj_vector[size-1];
      goals.push_back(lastPose);

      if (size > 1)
      {
        // Allow collision at start in case vehicle is (very) close to wall
        bool collision_allowed = true;

        for(int i = static_cast<int>(size-2); i >= 0; --i)
        {
          const geometry_msgs::PoseStamped& pose = traj_vector[i];
          unsigned int x,y;
          costmap_->worldToMap(pose.pose.position.x,pose.pose.position.y,x,y);
          unsigned int m_point = costmap_->getIndex(x,y);

          double dx = lastPose.pose.position.x - pose.pose.position.x;
          double dy = lastPose.pose.position.y - pose.pose.position.y;

          bool point_in_free_space = isFreeFrontiers(m_point);

          // extract points with 0.5m distance (if free)
          if(point_in_free_space)
          {
            if((dx*dx) + (dy*dy) > (0.25*0.25))
            {
              goals.push_back(pose);
              lastPose = pose;
              collision_allowed = false;
            }
          }

          if (!point_in_free_space && !collision_allowed)
          {
            break;
          }
        }
      }


      ROS_DEBUG("[hector_exploration_planner] pushed %u goals (trajectory) for inner frontier-search", (unsigned int)goals.size());

      // make exploration transform
      tf::Stamped<tf::Pose> robotPose;
      if(!costmap_ros_->getRobotPose(robotPose)){
        ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
      }
      geometry_msgs::PoseStamped robotPoseMsg;
      tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

      if (!buildexploration_trans_array_(robotPoseMsg, goals, false)){
        ROS_WARN("[hector_exploration_planner]: Creating exploration transform array in find inner frontier failed, aborting.");
        return false;
      }

      inner_vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());

      unsigned int x,y;
      costmap_->worldToMap(robotPoseMsg.pose.position.x,robotPoseMsg.pose.position.y,x,y);




      // get point with maximal distance to trajectory
      int max_exploration_trans_point = -1;
      unsigned int max_exploration_trans_val = 0;

      for(unsigned int i = 0; i < num_map_cells_; ++i){

        if(exploration_trans_array_[i] < UINT_MAX){
          if(exploration_trans_array_[i] > max_exploration_trans_val){
            if(!isFrontierReached(i)){
              max_exploration_trans_point = i;
              max_exploration_trans_val = exploration_trans_array_[i];
            }
          }
        }
      }

      if (max_exploration_trans_point == 0){
        ROS_WARN("[hector_exploration_planner]: Couldn't find max exploration transform point for inner exploration, aborting.");
        return false;
      }

      geometry_msgs::PoseStamped finalFrontier;
      unsigned int fx,fy;
      double wfx,wfy;
      costmap_->indexToCells(max_exploration_trans_point,fx,fy);
      costmap_->mapToWorld(fx,fy,wfx,wfy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wfx;
      finalFrontier.pose.position.y = wfy;
      finalFrontier.pose.position.z = 0.0;

      // assign orientation
      int dx = fx-x;
      int dy = fy-y;
      double yaw_path = std::atan2(dy,dx);
      finalFrontier.pose.orientation.x = 0.0;
      finalFrontier.pose.orientation.y = 0.0;
      finalFrontier.pose.orientation.z = sin(yaw_path*0.5f);
      finalFrontier.pose.orientation.w = cos(yaw_path*0.5f);

      innerFrontier.push_back(finalFrontier);

      if(visualization_pub_.getNumSubscribers() > 0){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "hector_exploration_planner";
        marker.id = 100;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = wfx;
        marker.pose.position.y = wfy;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_path);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(5,0);
        visualization_pub_.publish(marker);
      }
      return true;
    }
  }
  return false;
}

/*
 * checks if a given point is a frontier cell. a frontier cell is a cell in the occupancy grid
 * that seperates known from unknown space. Therefore the cell has to be free but at least three
 * of its neighbours need to be unknown
 * 判断一个给定的点是一个边缘细胞。边境上的一个细胞在栅格将已知的未知空间的一个细胞。
 * 因此，细胞必须是自由的，但至少有三的邻居需要未知。
 */

bool HectorExplorationPlanner::isFrontier(int point)//是不是边界
{
  if(isFreeFrontiers(point))
  {
    int adjacentPoints[8];//相邻的点（邻点 九宫格）
    getAdjacentPoints(point,adjacentPoints);
    //把point的八个方向的值赋给adjacentPoints的八个数组空位 如果越界啥的返回-1

    for(int i = 0; i < 8; ++i)
    {
      if(isValid(adjacentPoints[i]))//判断是否有效的  如果参数大于等于0 返回真
      {
        if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION)
        {
          //static const unsigned char NO_INFORMATION = 255;
          //NO_INFORMATION//const unsigned char* occupancy_grid_array_;
          int no_inf_count = 0;
          int noInfPoints[8];
          getAdjacentPoints(adjacentPoints[i],noInfPoints);
          for(int j = 0; j < 8; j++)
          {
            if( isValid(noInfPoints[j]) && occupancy_grid_array_[noInfPoints[j]] == costmap_2d::NO_INFORMATION)
            {
              ++no_inf_count;

              if(no_inf_count > 2)//如果找到三个（三个以上） 就可以返回 它是边界
              {
                return true;
              }
            }
          }
        }
      }
    }
  }

  return false;
}


void HectorExplorationPlanner::resetMaps()
{//重置地图

  
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  //把探索的路径的数组填成最大值
  std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
  //把障碍物的路径的数组填成最大值
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);

}

void HectorExplorationPlanner::clearFrontiers()
{//清理边界
  /**
   * std::fill函数的作用是：将一个区间的元素都赋予指定的值，即在[first, last)范围内填充指定值。 
   * std::fill_n函数的作用是：在[fist, fist + count)范围内填充指定值。
   * 这个函数把边界数组全置为了0
   */
  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);    
}

inline bool HectorExplorationPlanner::isValid(int point)
{
  return (point>=0);//是否在边界内 是否有效
}

bool HectorExplorationPlanner::isFree(int point)//是否空白的？？
{

  if(isValid(point))
  {
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free
    //如果一个点 它不是inscribed_inflated_obstacle，leathal_obstacle或no_information
    //那么他就是Free的
    if(p_use_inflated_obs_)//如果使用了膨胀半径
    {
      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return true;
      }
    } 
    else
    {
      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return true;
      }
    }

    if(p_plan_in_unknown_)//如果使用了在未知区域进行路径规划
    {
      if(occupancy_grid_array_[point] == costmap_2d::NO_INFORMATION)
      {
        return true;
      }
    }
  }
  return false;
}

bool HectorExplorationPlanner::isFreeFrontiers(int point)//是否空白的边界？？
{
  if(isValid(point))//if (point>=0) return yes
  {
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free

    if(p_use_inflated_obs_)//如果使用膨胀半径
    {
      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return true;
      }
    } 
    else 
    {
      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return true;
      }
    }
  }
  return false;
}

bool HectorExplorationPlanner::isFrontierReached(int point)//是否到达过这个边界
{
  tf::Stamped<tf::Pose> robotPose;
  if(!costmap_ros_->getRobotPose(robotPose))
  {
    ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
  }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

  unsigned int fx,fy;
  double wfx,wfy;
  costmap_->indexToCells(point,fx,fy);
  costmap_->mapToWorld(fx,fy,wfx,wfy);


  double dx = robotPoseMsg.pose.position.x - wfx;
  double dy = robotPoseMsg.pose.position.y - wfy;

  if ( (dx*dx) + (dy*dy) < (p_dist_for_goal_reached_*p_dist_for_goal_reached_)) 
  {
    ROS_DEBUG("[hector_exploration_planner]: frontier is within the squared range of: %f", p_dist_for_goal_reached_);
    return true;
  }
  return false;

}

bool HectorExplorationPlanner::isSameFrontier(int frontier_point1, int frontier_point2)
{   //判断是否是相同的边界

  unsigned int fx1,fy1;
  unsigned int fx2,fy2;
  double wfx1,wfy1;
  double wfx2,wfy2;
  costmap_->indexToCells(frontier_point1,fx1,fy1);//转化为x y坐标
  costmap_->indexToCells(frontier_point2,fx2,fy2);
  costmap_->mapToWorld(fx1,fy1,wfx1,wfy1);//地图坐标系 转化为 世界坐标系
  costmap_->mapToWorld(fx2,fy2,wfx2,wfy2);

  double dx = wfx1 - wfx2;
  double dy = wfy1 - wfy2;

  if((dx*dx) + (dy*dy) < (p_same_frontier_dist_*p_same_frontier_dist_))
  {//config 传入 一开始是0.25 
    return true;
  }
  return false;
}



//??????????????????????????????????????????????????????????????????????
inline unsigned int HectorExplorationPlanner::cellDanger(int point)
{

  if ((int)obstacle_trans_array_[point] <= p_min_obstacle_dist_)
  {
    return static_cast<unsigned int>(p_alpha_ * std::pow(p_min_obstacle_dist_ - obstacle_trans_array_[point], 2) + .5);
  }
  //ROS_INFO("%d", (int)obstacle_trans_array_[point] );
  //return 80000u - std::min(80000u, obstacle_trans_array_[point]*40);

  //return (2000u - std::min(2000u, obstacle_trans_array_[point])) / 500u;
  //std::cout << obstacle_trans_array_[point] << "\n";

  return 0;
}


//角差 这个函数计算的是我 当前相对于坐标系的角度 和 当前点和目标点的之间的角度差 然后应该是用来给车转动方向
float HectorExplorationPlanner::angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
  // setup start positions 设置起始位置
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);
  //将开始点的世界坐标系转化到地图坐标系

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);
  //将目标点的世界坐标系转化到地图坐标系  

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle)
  {
    both_angle = start_angle - goal_angle;
  } 
  else 
  {
    both_angle = goal_angle - start_angle;
  }

  if(both_angle > M_PI)
  {
    both_angle = (M_PI - std::abs(start_angle)) + (M_PI - std::abs(goal_angle));
  }

  return both_angle;
}

// Used to generate direction for frontiers 用于产生前沿的方向
//这个函数 得到point这个点 到周围八邻域中的 最大的那个No_information定位点的角度
double HectorExplorationPlanner::getYawToUnknown(int point)
{
  int adjacentPoints[8];//邻点 
  getAdjacentPoints(point,adjacentPoints);//得到八个方向的八个点

  int max_obs_idx = 0;
  unsigned int max_obs_dist = 0;

  for(int i = 0; i < 8; ++i)//遍历八个方向
  {
    if(isValid(adjacentPoints[i]))//如果在范围内 >= 0
    {
      if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION)
      {//如果栅格地图的概率值等于 未知 255
        if(obstacle_trans_array_[adjacentPoints[i]] > max_obs_dist)
        {
          max_obs_idx = i;
          max_obs_dist = obstacle_trans_array_[adjacentPoints[i]];
        }
      }
    }
  }//经过循环 找到了 该点周围八个点中是No_information的点中 最大的点

  int orientationPoint = adjacentPoints[max_obs_idx];//orientation Point定位点
  unsigned int sx,sy,gx,gy;
  costmap_->indexToCells((unsigned int)point,sx,sy);//当前的点
  costmap_->indexToCells((unsigned int)orientationPoint,gx,gy);//八邻域中 定位点.

  int x = gx-sx;
  int y = gy-sy;
  double yaw = std::atan2(y,x);
  //atan2是一个函数，在C语言里返回的是指方位角，也可以理解为计算复数 x+yi 的辐角，计算时atan2 比 atan 稳定。    返回的是原点至点(x,y)的方位角，即与 x 轴的夹角

  return yaw;

}

unsigned int HectorExplorationPlanner::angleDanger(float angle)
{
  float angle_fraction = std::pow(angle,3);///M_PI;

  unsigned int result = static_cast<unsigned int>(p_goal_angle_penalty_ * angle_fraction);

  return result;
}

float HectorExplorationPlanner::getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2)
{//得到距离权重

  float distance = std::sqrt(std::pow(point1.pose.position.x - point2.pose.position.x,2) + std::pow(point1.pose.position.y - point2.pose.position.y,2));

  if(distance < 0.5)
  {
    return 5.0;
  } 
  else 
  {
    return 1;
  }
}

/*
 These functions calculate the index of an adjacent point (left,upleft,up,upright,right,downright,down,downleft) to the
 given point. If there is no such point (meaning the point would cause the index to be out of bounds), -1 is returned.
 */
inline void HectorExplorationPlanner::getStraightPoints(int point, int points[])
{
  //得到八邻域的上下左右
  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);

}

inline void HectorExplorationPlanner::getDiagonalPoints(int point, int points[])
{ 
  //得到八邻域的四个角
  points[0] = upleft(point);
  points[1] = upright(point);
  points[2] = downright(point);
  points[3] = downleft(point);

}

/*
inline void HectorExplorationPlanner::getStraightAndDiagonalPoints(int point, int straight_points[], int diag_points[]){
  /
  // Can go up if index is larger than width
  bool up = (point >= (int)map_width_);

  // Can go down if
  bool down = ((point/map_width_) < (map_width_-1));


  bool right = ((point + 1) % map_width_ != 0);
  bool left = ((point % map_width_ != 0));

}
*/

inline void HectorExplorationPlanner::getAdjacentPoints(int point, int points[])
{
  //得到八邻域
  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);

}

inline int HectorExplorationPlanner::left(int point)
{
  // only go left if no index error and if current point is not already on the left boundary 只有在没有索引错误的情况下才向左，如果当前点还没有在左边界上。
  if((point % map_width_ != 0))
  {
    return point-1;
  }
  return -1;
}
inline int HectorExplorationPlanner::upleft(int point)
{
  if((point % map_width_ != 0) && (point >= (int)map_width_))
  {
    return point-1-map_width_;
  }
  return -1;

}
inline int HectorExplorationPlanner::up(int point)
{
  if(point >= (int)map_width_)
  {
    return point-map_width_;
  }
  return -1;
}
inline int HectorExplorationPlanner::upright(int point)
{
  if((point >= (int)map_width_) && ((point + 1) % (int)map_width_ != 0))
  {
    return point-map_width_+1;
  }
  return -1;
}
inline int HectorExplorationPlanner::right(int point)
{
  if((point + 1) % map_width_ != 0)
  {
    return point+1;
  }
  return -1;

}
inline int HectorExplorationPlanner::downright(int point)
{
  if(((point + 1) % map_width_ != 0) && ((point/map_width_) < (map_height_-1)))
  {
    return point+map_width_+1;
  }
  return -1;

}
inline int HectorExplorationPlanner::down(int point)
{
  if((point/map_width_) < (map_height_-1))
  {
    return point+map_width_;
  }
  return -1;

}
inline int HectorExplorationPlanner::downleft(int point)
{
  if(((point/map_width_) < (map_height_-1)) && (point % map_width_ != 0))
  {
    return point+map_width_-1;
  }
  return -1;

}

//        // visualization (export to another method?)
//        visualization_msgs::Marker marker;
//        marker.header.frame_id = "map";
//        marker.header.stamp = ros::Time();
//        marker.ns = "hector_exploration_planner";
//        marker.id = i + 500;
//        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//        marker.action = visualization_msgs::Marker::ADD;
//        marker.pose.position = goals[i].pose.position;
//        marker.scale.x = 0.2;
//        marker.scale.y = 0.2;
//        marker.scale.z = 0.2;
//        marker.color.a = 1.0;
//        marker.color.r = 0.0;
//        marker.color.g = 0.0;
//        marker.color.b = 1.0;
//        marker.lifetime = ros::Duration(5,0);
//        marker.text = boost::lexical_cast<std::string>((int)init_cost) + " - " + boost::lexical_cast<std::string>(getDistanceWeight(start,goals[i]));
//        visualization_pub_.publish(marker);

//void HectorExplorationPlanner::saveMaps(std::string path){

//    char costmapPath[1000];
//    sprintf(costmapPath,"%s.map",path.data());
//    char explorationPath[1000];
//    sprintf(explorationPath,"%s.expl",path.data());
//    char obstaclePath[1000];
//    sprintf(obstaclePath,"%s.obs",path.data());
//    char frontierPath[1000];
//    sprintf(frontierPath,"%s.front",path.data());


//    costmap.saveMap(costmapPath);
//    FILE *fp_expl = fopen(explorationPath,"w");
//    FILE *fp_obs = fopen(obstaclePath,"w");
//    FILE *fp_front = fopen(frontierPath,"w");

//    if (!fp_expl || !fp_obs || !fp_front)
//    {
//        ROS_WARN("[hector_exploration_planner] Cannot save maps");
//        return;
//    }

//    for(unsigned int y = 0; y < map_height_; ++y){
//        for(unsigned int x = 0;x < map_width_; ++x){
//            unsigned int expl = exploration_trans_array_[costmap.getIndex(x,y)];
//            unsigned int obs = obstacle_trans_array_[costmap.getIndex(x,y)];
//            int blobVal = frontier_map_array_[costmap.getIndex(x,y)];
//            fprintf(fp_front,"%d\t", blobVal);
//            fprintf(fp_expl,"%d\t", expl);
//            fprintf(fp_obs, "%d\t", obs);
//        }
//        fprintf(fp_expl,"\n");
//        fprintf(fp_obs,"\n");
//        fprintf(fp_front,"\n");
//    }

//    fclose(fp_expl);
//    fclose(fp_obs);
//    fclose(fp_front);
//    ROS_INFO("[hector_exploration_planner] Maps have been saved!");
//    return;

//}

//    // add last point to path (goal point)
//    for(unsigned int i = 0; i < goals.size(); ++i){
//        unsigned int mx,my;
//        costmap.worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my);

//        if(currentPoint == (int)costmap.getIndex(mx,my)){
//            plan.push_back(goals[i]);
//            previous_goal_ = currentPoint;
//        }

//    }

