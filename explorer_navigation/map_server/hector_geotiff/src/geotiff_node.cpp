/**
 * 保存并绘制geotiff类型地图的节点
 * 设置了背景的颜色 存储路径 图片名字 绘制棋盘背景 绘制网格
 * 是一个client通过“map”获得地图数据
 * 是一个client通过“trajectory”客户端获得路径的数据  这个客户端的service在hector_trajectory_server
 * 订阅系统命令 如果系统命令等于“savegeotiff”，节点将保存一个geotiff文件。
 * 这个代码好像使用的一个回调函数使这个保存的命令进行
 * 设定多时间保存一次地图 这个通过launch文件传入
 * 订阅插件列表 列表在launch文件中给出
 * 循环将插件列表读入然后逐一进行插件的调用
 * 在writeGeotiff的时候 获取存储的时间 存储地图的名称 比赛名称 队伍名称 任务名称等
 * 
 */

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <boost/algorithm/string.hpp>
#include "nav_msgs/GetMap.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include <Eigen/Geometry>
#include <QtGui/QApplication>
#include <hector_map_tools/HectorMapTools.h>
#include <hector_geotiff/geotiff_writer.h>
#include <hector_geotiff/map_writer_plugin_interface.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>

using namespace std;

namespace hector_geotiff
{

  /**
   * Map generation node.  geotiff.node由launch文件打开
   */
  class MapGenerator //地图生产者
  {
  public:
    MapGenerator()
        : geotiff_writer_(false) 
        // 这边设置了false -> background -> white. 白底色
        //         true -> background -> black   黑底色
          ,
          pn_("~"), plugin_loader_(0), running_saved_map_num_(0)
    {
      //在launch文件中  <arg name="map_file_path" default="/home/sun/catkin_slam/maps"/>
      pn_.param("map_file_path", p_map_file_path_, std::string("."));
     
      geotiff_writer_.setMapFilePath(p_map_file_path_);//设置存储路径
     
      geotiff_writer_.setUseUtcTimeSuffix(true);
      //这个接口用于是否使用我们自己给图定名字，还是使用当前的时间

      pn_.param("map_file_base_name", p_map_file_base_name_, std::string());
     
      pn_.param("draw_background_checkerboard", p_draw_background_checkerboard_, true); //draw_background_checkerboard绘制背景棋盘 true给1
     
      pn_.param("draw_free_space_grid", p_draw_free_space_grid_, true);
      //draw free space grid绘制自由空间网格 true给1

      sys_cmd_sub_ = n_.subscribe("syscommand", 1, &MapGenerator::sysCmdCallback, this);
      //系统命令。 如果字符串等于“savegeotiff”，节点将保存一个geotiff文件。

      map_service_client_ = n_.serviceClient<nav_msgs::GetMap>("map");
      //一个服务的客户端 通过call("map") 向hector_map_server发送命令请求得到地图的命令

      //object_service_client_ = n_.serviceClient<worldmodel_msgs::GetObjectModel>("worldmodel/get_object_model");

      path_service_client_ = n_.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");
      //一个call “trajectory”话题的客户端 这个客户端的service在hector_trajectory_server

      ///ROS_INFO("IN geotiff_node.cpp   1");
      ///ROS_ERROR("IN geotiff_node.cpp   %s",p_map_file_base_name_.c_str());

      double p_geotiff_save_period = 0.0; //每多长时间保存一次地图　这个在launch文件中给定了
      pn_.param("geotiff_save_period", p_geotiff_save_period, 0.0);

      if (p_geotiff_save_period > 0.0)
      {
        //ros::Timer timer = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, false);
        //publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), &PathContainer::publishTrajectoryTimerCallback, this, false);
        map_save_timer_ = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, this, false);
          //在这里调用保存地图的回调函数
      }

      pn_.param("plugins", p_plugin_list_, std::string(""));

      std::vector<std::string> plugin_list;

      boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));


      /**
       * We always have at least one element containing "" in the string list
       * 在字符串列表中，我们至少有一个包含“”的元素。 
       * 然后循环将插件按顺序存入一个vector plugin_vector_.push_back(tmp) 后边调用
       */
      if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0))
      {
        plugin_loader_ = new pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>("hector_geotiff", "hector_geotiff::MapWriterPluginInterface");

        for (size_t i = 0; i < plugin_list.size(); ++i)
        {
          try
          {
            boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> tmp(plugin_loader_->createInstance(plugin_list[i]));

            tmp->initialize(plugin_loader_->getName(plugin_list[i]));

            plugin_vector_.push_back(tmp);
          }
          catch (pluginlib::PluginlibException &ex)
          {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
          }
        }
      }
      else
      {
        ROS_INFO("No plugins loaded for geotiff node");
      }

      ROS_WARN("Geotiff node started, successed init\n\n");
    }//初始化

    ~MapGenerator()
    {
      if (plugin_loader_)
      {
        delete plugin_loader_;
      }
    }

    void writeGeotiff()
    {
      ros::Time start_time(ros::Time::now());//获取当前时间
      std::stringstream ssStream; //定义一个字符串数组
      ///ROS_ERROR("In geotiff_node.cpp  in writeGeotiff()  first!!!");

      nav_msgs::GetMap srv_map;//定义一个新的地图类型的变量 用来存储通过话题订阅 传入的地图数据

      if (map_service_client_.call(srv_map))//从”map“话题里得到地图的数据
      {
        /**
         * when we use 'rostopic pub syscommand std_msgs/String "savegeotiff" .. 
         * the codes after here will run'
         * 定义了一个OccupancyGrid类型的map 得到了call “map” 节点 得到的地图
         * map_service_client_ = n_.serviceClient<nav_msgs::GetMap>("map");
         * 个服务的客户端 通过call("map") 向server发送命令请求得到地图的命令
         * 
         */
        ROS_INFO("\n\nGeotiffNode: Call map service to acquire map successfully");
        const nav_msgs::OccupancyGrid &map(srv_map.response.map);//map在这 得到的回复存到map了


        std::string map_file_name = p_map_file_base_name_;//存储地图的位置
        std::string competition_name;//比赛名称？
        std::string team_name;//队伍名称？
        std::string mission_name;//任务名称
        std::string postfix;//后缀

        if (n_.getParamCached("/competition", competition_name) && !competition_name.empty())
          map_file_name = map_file_name + "_" + competition_name;
        if (n_.getParamCached("/team", team_name) && !team_name.empty())
          map_file_name = map_file_name + "_" + team_name;
        if (n_.getParamCached("/mission", mission_name) && !mission_name.empty())
          map_file_name = map_file_name + "_" + mission_name;
        if (pn_.getParamCached("map_file_postfix", postfix) && !postfix.empty())
          map_file_name = map_file_name + "_" + postfix;
        if (map_file_name.substr(0, 1) == "_")
          map_file_name = map_file_name.substr(1);
        if (map_file_name.empty())
          map_file_name = "GeoTiffMap";





        geotiff_writer_.setMapFileName(map_file_name);//设置Map名称

        bool transformSuccess = geotiff_writer_.setupTransforms(map);//进行tf变 ？？？map是谁

        if (!transformSuccess)//如果tf变换没有成功 输出错误信息 返回
        {
          ROS_INFO("GeotiffNode: Couldn't set map transform");
          return;
        }

        geotiff_writer_.setupImageSize(); //设置图像大小？
        

        if (p_draw_background_checkerboard_)//前方给了初值1 是否设置背景棋盘
        {                                              
          geotiff_writer_.drawBackgroundCheckerboard(); //绘制绘制背景棋盘 衬托背景的那个棋盘背景
        }

        geotiff_writer_.drawMap(map, p_draw_free_space_grid_); 
        //绘制地图   p_draw_free_space_grid_前边给的默认值为1 

        geotiff_writer_.drawCoords();//绘制坐标 左上角的东西

        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      }//end if 
      else//如果call没有得到回复
      {
        ROS_ERROR("Failed to call map service");
        return;
      }
      
      /**
       * @brief 下面开始引用plugin 
       */

      ROS_ERROR("In hector_geotiff  geotiff_node.cpp \n\
                Start to use plugin . \n\
                If QR and Victim plugin all used you will see two lines of [USE_PLUGIN] ");
      for (size_t i = 0; i < plugin_vector_.size(); ++i)
      {
        plugin_vector_[i]->draw(&geotiff_writer_);
        //这里就开始调用插件里边的函数 worldmodel_geotiff_plugins.cpp
        ROS_ERROR("In hector_geotiff   geotiff_node.cpp   USE_PLUGIN");
      }

      /**
        * No Victims for now, first  agree on a common standard for representation
        目前没有受害者，首先就共同的代表标准达成一致意见。
        */
      /*
      if (req_object_model_){
        worldmodel_msgs::GetObjectModel srv_objects;
        if (object_service_client_.call(srv_objects))
        {
          ROS_INFO("GeotiffNode: Object service called successfully");

          const worldmodel_msgs::ObjectModel& objects_model (srv_objects.response.model);

          size_t size = objects_model.objects.size();


          unsigned int victim_num  = 1;

          for (size_t i = 0; i < size; ++i){
            const worldmodel_msgs::Object& object (objects_model.objects[i]);

            if (object.state.state == worldmodel_msgs::ObjectState::CONFIRMED){
              geotiff_writer_.drawVictim(Eigen::Vector2f(object.pose.pose.position.x,object.pose.pose.position.y),victim_num);
              victim_num++;
            }
          }
        }
        else
        {
          ROS_ERROR("Failed to call objects service");
        }
      }
      */
      
    /*
      下面这个部分具体作用是什么我并不明白，好像是保持之前的轨迹，如果你的object_tracker重启了
      执行hector_trajectory之后，会有trajectory这个service
    */
      /*这一段被开启了
      hector_nav_msgs::GetRobotTrajectory srv_path;

      if (path_service_client_.call(srv_path))
      {
        ROS_INFO("GeotiffNode: Path service called successfully");

        std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

        size_t size = traj_vector.size();

        std::vector<Eigen::Vector2f> pointVec;
        pointVec.resize(size);

        for (size_t i = 0; i < size; ++i){
          const geometry_msgs::PoseStamped& pose (traj_vector[i]);

          pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
        }

        if (size > 0){
          //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
          Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
          geotiff_writer_.drawPath(startVec, pointVec);
        }
      }
      else
      {
        ROS_ERROR("Failed to call path service");
      }
      */

      geotiff_writer_.writeGeotiffImage();
      running_saved_map_num_++;
      ROS_WARN("geotiff have write %d maps",running_saved_map_num_);

      ros::Duration elapsed_time(ros::Time::now() - start_time);

      ROS_INFO("GeoTiff created in %f seconds", elapsed_time.toSec());
    
    }//end writeGeoTiff 

    void timerSaveGeotiffCallback(const ros::TimerEvent &e)
    {
      this->writeGeotiff();
      //这个函数的意思是不是不用接受系统的savegeotiff命令也能进行保存
    }

    void sysCmdCallback(const std_msgs::String &sys_cmd)
    {
      /**
       * @brief 订阅系统命令 如果系统命令等于"savegeotiff" 则调用write函数 绘制保存地图
       * 
       */
      if (!(sys_cmd.data == "savegeotiff"))
      {
        return;
      }

      this->writeGeotiff();
    }

    std::string p_map_file_path_;
    std::string p_map_file_base_name_;
    std::string p_plugin_list_;
    bool p_draw_background_checkerboard_;
    bool p_draw_free_space_grid_;

    //double p_geotiff_save_period_;

    GeotiffWriter geotiff_writer_;

    ros::ServiceClient map_service_client_; // = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    ros::ServiceClient object_service_client_;
    ros::ServiceClient path_service_client_;

    ros::Subscriber sys_cmd_sub_;

    ros::NodeHandle n_;
    ros::NodeHandle pn_;

    std::vector<boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> > plugin_vector_;

    pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface> *plugin_loader_;

    ros::Timer map_save_timer_;

    unsigned int running_saved_map_num_;
  };
}//namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geotiff_node");
  //ROS_INFO("In geotiff_node.cpp  2");
  hector_geotiff::MapGenerator mg;
  //ROS_INFO("In geotiff_node.cpp 3");

  //ros::NodeHandle pn_;
  //double p_geotiff_save_period = 60.0f;
  //pn_.param("geotiff_save_period", p_geotiff_save_period, 60.0);
  //ros::Timer timer = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, &mg, false);

  ros::spin();

  return 0;
}
