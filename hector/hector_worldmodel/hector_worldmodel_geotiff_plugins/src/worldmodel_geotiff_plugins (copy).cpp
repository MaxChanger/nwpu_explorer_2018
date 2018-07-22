
#include <hector_geotiff/map_writer_interface.h>
#include <hector_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
//#include <hector_worldmodel_msgs/AddVictim.h>

#include <pluginlib/class_loader.h>
#include <fstream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/formatters.hpp>

#include <boost/tokenizer.hpp>

namespace hector_worldmodel_geotiff_plugins {

using namespace hector_geotiff;

class MapWriterPlugin : public MapWriterPluginInterface
{
public:
  MapWriterPlugin();
  virtual ~MapWriterPlugin();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface) = 0;

protected:

  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;
//  ros::ServiceClient service_add_victim_client_;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;


// 新加这些部分
  std::string team_name;
  std::string country;
  std::string mission_name;
  
  ros::Time last_time;
  Eigen::Vector2f last_pose;

  double time_torlerence_;
  double dist_torlerence_;

};

MapWriterPlugin::MapWriterPlugin()
  : initialized_(false)
{}

MapWriterPlugin::~MapWriterPlugin()
{}

void MapWriterPlugin::initialize(const std::string& name)
{ 
  ROS_WARN("in the initation");
  ros::NodeHandle plugin_nh("~/" + name); std::string service_name_;

  std::string service_add_victim_; // your service name

//plugin_nh.param("service_name", service_name_, std::string("worldmodel/get_object_model")); // 这个之前在hector_object_tracker 以及 hector_map_server 中都是有的，那边随便改名
  plugin_nh.param("service_name",service_name_,std::string("/get_object_model"));    // 这个是自己加的

//  plugin_nh.param("service_add_victim",service_add_victim_, std::string("/srv_add_victim") ); // to add victim

  plugin_nh.param("draw_all_objects", draw_all_objects_, false);
  plugin_nh.param("class_id", class_id_, std::string());

  plugin_nh.param("team_name", team_name , std::string() );  
  plugin_nh.param("country" , country , std::string() );
  plugin_nh.param("mission_name", mission_name , std::string() );
  plugin_nh.param("same_pose_time_torlerence", time_torlerence_, 0.1);
  plugin_nh.param("same_pose_torlerence", dist_torlerence_, 0.6);

  service_client_ = nh_.serviceClient<hector_worldmodel_msgs::GetObjectModel>(service_name_);
//  service_add_victim_client_ = nh_.serviceClient<hector_worldmodel_msgs::AddVictim>(service_add_victim_);
  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized hector_geotiff MapWriter plugin %s.", name_.c_str());
  last_pose.x() = 0;
  last_pose.y() = 0;
  last_time = ros::Time(0);
}



class VictimMapWriter : public MapWriterPlugin
{
public:
  void initialize(const std::string& name)
  {
    MapWriterPlugin::initialize(name);
    ROS_WARN("rebuiled the service");
    service_client_ = nh_.serviceClient<hector_worldmodel_msgs::GetObjectModel>("/get_victim_models");
  }

  virtual ~VictimMapWriter() {}

  void draw(MapWriterInterface *interface)
  {
    //ROS_WARN("in the victim writer!");
    last_pose.x()=0;
    last_pose.y()=0;
    if (!initialized_) return;

    hector_worldmodel_msgs::GetObjectModel data;
    if (!service_client_.call(data)) {
      ROS_ERROR_NAMED(name_, "Cannot draw victims, service %s failed", service_client_.getService().c_str());
      return;
    }

/*
    std::string team_name;
    std::string country;
    std::string mission_name;
    nh_.getParamCached("/team", team_name);
    nh_.getParamCached("/country", country);
    nh_.getParamCached("/mission", mission_name);
*/

    boost::posix_time::ptime now = ros::Time::now().toBoost();
    boost::gregorian::date now_date(now.date());
    boost::posix_time::time_duration now_time(now.time_of_day().hours(), now.time_of_day().minutes(), now.time_of_day().seconds(), 0);

    std::ofstream description_file((interface->getBasePathAndFileName() + "_victims.csv").c_str());
    if (description_file.is_open()) {
      description_file << "\"victims\"" << std::endl;
      description_file << "\"1.0\"" << std::endl;
      if (!team_name.empty()) description_file << "\"" << team_name << "\"" << std::endl;
      if (!country.empty()) description_file << "\"" << country << "\"" << std::endl;
      description_file << "\"" << now_date << "\"" << std::endl;
      description_file << "\"" << now_time << "\"" << std::endl;
      if (!mission_name.empty()) description_file << "\"" << mission_name << "\"" << std::endl;
      description_file << std::endl;
      description_file << "id,time,name,x,y,z" << std::endl;
    }

    //ROS_ERROR("In worldmodel_geotiff_plugins.cpp, Start to loop the all probable victims.");

    int counter = 0;
    ROS_WARN("in draw start:now the size:%d",data.response.model.objects.size());
    for(hector_worldmodel_msgs::ObjectModel::_objects_type::const_iterator it = data.response.model.objects.begin();
        it != data.response.model.objects.end();++it) 
  {

        //ROS_ERROR("In worldmodel_geotiff_plugins.cpp, In <loop the all probable victims.> ");

      const hector_worldmodel_msgs::Object& object = *it;
      if (!draw_all_objects_ && object.state.state != hector_worldmodel_msgs::ObjectState::CONFIRMED) 
      {
        ROS_ERROR("in worldmodel_geotiff_plugins.cpp 1");
        continue;
      }

      if (!class_id_.empty() && object.info.class_id != class_id_)
      {
        ROS_ERROR("1. %s",class_id_.c_str());
        ROS_ERROR("2. %s",object.info.class_id.c_str());
        ROS_ERROR("In worldmodel_geotiff_plugins.cpp  if (!class_id_.empty() && object.info.class_id != class_id_)");
        continue;
      }


// add by myself
//
//      hector_worldmodel_msgs::AddVictim  addvictim;
//      addvictim.request.req = "addvictimservice";
//
//      if (!service_add_victim_client_.call(addvictim))
//      {
//        ROS_ERROR("Get The Response : [%s]", addvictim.response.res.c_str());
//        continue;
//      }
//      else{
//        ROS_ERROR("Get The Response : [%s]", addvictim.response.res.c_str() );
//      }

/*
*   add victims into geotiff. and add the victim's information into the <>.csv file.
*/
      Eigen::Vector2f coords;
      coords.x() = object.pose.pose.position.x;
      coords.y() = object.pose.pose.position.y;
	  
      // 增加数字编号的就是 boost::lexical_cast<std::string>(++counter)
      double diff = 0;
      diff = (last_pose.x() - coords.x()) * (last_pose.x() - coords.x()) + (last_pose.y() - coords.y()) * (last_pose.y() - coords.y()); 
      ROS_WARN("curr_x %f,curr_y %f,distance: %f",coords.x(),coords.y(),diff);
	    if ( diff > dist_torlerence_ * dist_torlerence_ /*&& last_time + ros::Duration(time_torlerence_) > ros::Time::now()*/ ) 
      { 
        interface->drawObjectOfInterest(coords, boost::lexical_cast<std::string>(++counter), MapWriterInterface::Color(240,10,10));

		    //last_time = ros::Time::now();
		    last_pose.x() = coords.x();
		    last_pose.y() = coords.y();

        if (description_file.is_open()) 
        {
          boost::posix_time::time_duration time_of_day(object.header.stamp.toBoost().time_of_day());
          boost::posix_time::time_duration time(time_of_day.hours(), time_of_day.minutes(), time_of_day.seconds(), 0);
          description_file << counter << "," << time << "," << object.info.object_id << "," << object.pose.pose.position.x << "," << object.pose.pose.position.y << "," << object.pose.pose.position.z << std::endl;
        } // end of if
        ROS_WARN("You add victim!");
	    }
    }//end of for
    //ROS_WARN("in draw end:now the size:%d",data.response.model.objects.size());
  }// end of draw
};

class QRCodeMapWriter : public MapWriterPlugin
{
public:
  virtual ~QRCodeMapWriter() {}

  void draw(MapWriterInterface *interface)
  {
    if (!initialized_) return;

    hector_worldmodel_msgs::GetObjectModel data;
    if (!service_client_.call(data)) {
      ROS_ERROR_NAMED(name_, "Cannot draw victims, service %s failed", service_client_.getService().c_str());
      return;
    }

/*
* 下面这些是原来的部分，但是在launch中修改完全没有效果
   尚不明白原因
    std::string team_name;
    std::string country;
    std::string mission_name;

    nh_.getParamCached("/team", team_name);
    nh_.getParamCached("/country", country);
    nh_.getParamCached("/mission", mission_name);

*/

// 当我们使用stage_ros来模拟时，stage_ros提供的时间是虚拟时间，也就是说，是从0开始的。
    boost::posix_time::ptime now = ros::Time::now().toBoost();
    boost::gregorian::date now_date(now.date());
    boost::posix_time::time_duration now_time(now.time_of_day().hours(), now.time_of_day().minutes(), now.time_of_day().seconds(), 0);

    std::ofstream description_file((interface->getBasePathAndFileName() + "_qr.csv").c_str());
    if (description_file.is_open()) {
      description_file << "\"qr codes\"" << std::endl;
      description_file << "\"1.0\"" << std::endl;
      if (!team_name.empty()) description_file << "\"" << team_name << "\"" << std::endl;
      if (!country.empty()) description_file << "\"" << country << "\"" << std::endl;
      description_file << "\"" << now_date << "\"" << std::endl;
      description_file << "\"" << now_time << "\"" << std::endl;
      if (!mission_name.empty()) description_file << "\"" << mission_name << "\"" << std::endl;
      description_file << std::endl;
      description_file << "id,time,text,x,y,z" << std::endl;
    }


    int counter = 0;
    for(hector_worldmodel_msgs::ObjectModel::_objects_type::const_iterator it = data.response.model.objects.begin(); it != data.response.model.objects.end(); ++it) {
      const hector_worldmodel_msgs::Object& object = *it;
      if (!class_id_.empty() && object.info.class_id != class_id_) continue;
      if (!draw_all_objects_ && object.state.state != hector_worldmodel_msgs::ObjectState::CONFIRMED) continue;
      if (object.state.state == hector_worldmodel_msgs::ObjectState::DISCARDED) continue;

      ++counter;

      // add only largest qr codes into geotiff
      if (isLargest(object, data.response.model.objects)) {
          Eigen::Vector2f coords;
          coords.x() = object.pose.pose.position.x;
          coords.y() = object.pose.pose.position.y;
          interface->drawObjectOfInterest(coords, boost::lexical_cast<std::string>(counter), MapWriterInterface::Color(10,10,240));
      }

      if (description_file.is_open()) {
        boost::posix_time::time_duration time_of_day(object.header.stamp.toBoost().time_of_day());
        boost::posix_time::time_duration time(time_of_day.hours(), time_of_day.minutes(), time_of_day.seconds(), 0);
        description_file << counter << "," << time << "," << object.info.name << "," << object.pose.pose.position.x << "," << object.pose.pose.position.y << "," << object.pose.pose.position.z << std::endl;
      }
    }

    description_file.close();
  }

protected:
  float getSizeFromName(const std::string& name)
  {
    try {
      boost::tokenizer<boost::char_separator<char> > tokens(name, boost::char_separator<char>("_"));
      boost::tokenizer<boost::char_separator<char> >::const_iterator it = tokens.begin();

      if (it == tokens.end())
        return -1.0f;

      for (unsigned int i = 0; i < 3; i++) {
        if (++it == tokens.end())
          return -1.0f;
      }

      return it->size() > 2 ? boost::lexical_cast<float>(it->substr(0, it->size()-2)) : -1.0f;
    }
    catch (boost::bad_lexical_cast&) {
      return -1.0f;
    }
  }

  bool isLargest(const hector_worldmodel_msgs::Object& object, const std::vector<hector_worldmodel_msgs::Object>& objects )
  {
    // determine size of qr code
    float size = getSizeFromName(object.info.name);

    if (size == -1.0f) // QR does not include size information
    {
      ROS_ERROR("In worldmodel_geotiff_plugins.cpp  _function_isLargest_if_size_==_-1.0f_ ");
      return true;
    }

    // compare size of other qr codes
    for (hector_worldmodel_msgs::ObjectModel::_objects_type::const_iterator it = objects.begin(); it != objects.end(); ++it) {
      const hector_worldmodel_msgs::Object& object2 = *it;

      float dist_sqr = (object.pose.pose.position.x-object2.pose.pose.position.x) * (object.pose.pose.position.x-object2.pose.pose.position.x)
                      +(object.pose.pose.position.y-object2.pose.pose.position.y) * (object.pose.pose.position.y-object2.pose.pose.position.y);

      // check if both qrcodes are at same position+tolerance
      if (dist_sqr < 0.75f) {
        float size2 = getSizeFromName(object2.info.name);
        if (size2 > size) {
          return false;
        }
      }
    }

    return true;
  }
};

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_worldmodel_geotiff_plugins::VictimMapWriter, hector_geotiff::MapWriterPluginInterface)
PLUGINLIB_EXPORT_CLASS(hector_worldmodel_geotiff_plugins::QRCodeMapWriter, hector_geotiff::MapWriterPluginInterface)
PLUGINLIB_EXPORT_CLASS(hector_worldmodel_geotiff_plugins::QRCodeMapWriter, hector_geotiff::MapWriterPluginInterface)
