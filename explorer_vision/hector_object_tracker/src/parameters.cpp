

#include "parameters.h"
#include <set>

#include <ros/node_handle.h>
#include <ros/console.h>

#include <hector_nav_msgs/GetDistanceToObstacle.h>
#include <hector_nav_msgs/GetNormal.h>

namespace hector_object_tracker {
  
std::map<std::string, bool>   _project_objects;
std::map<std::string, bool>   _with_orientation;
std::map<std::string, double> _default_distance;
std::map<std::string, double> _distance_variance;
std::map<std::string, double> _angle_variance;
std::map<std::string, double> _min_height;
std::map<std::string, double> _max_height;
std::map<std::string, double> _pending_support;
std::map<std::string, double> _pending_time;
std::map<std::string, double> _active_support;
std::map<std::string, double> _active_time;
std::map<std::string, double> _inactive_support;
std::map<std::string, double> _inactive_time;
std::map<std::string, double> _min_distance_between_objects;

std::map<std::string, std_msgs::ColorRGBA> _marker_color;
std::map<std::string, ros::ServiceClientPtr> _distance_to_obstacle_service;
std::map<std::string, ros::ServiceClientPtr> _get_normal_service;
std::map<std::string, ServiceClientsWithProperties> _percept_verification_services;
std::map<std::string, ServiceClientsWithProperties> _object_verification_services;

namespace Parameters {

  void load(const std::string &class_id /* = std::string() */)
  {
    static std::set<std::string> _per_class_parameters_loaded;

    if (_per_class_parameters_loaded.count(class_id)) return;


    ros::NodeHandle priv_nh("~");

    std::string prefix = class_id + "/";
    if (prefix == "/") prefix.clear();

    if (priv_nh.hasParam(prefix + "project_objects"))
      priv_nh.getParam(prefix + "project_objects", _project_objects[class_id]);
    if (priv_nh.hasParam(prefix + "with_orientation"))
      priv_nh.getParam(prefix + "with_orientation", _with_orientation[class_id]);
    if (priv_nh.hasParam(prefix + "default_distance"))
      priv_nh.getParam(prefix + "default_distance", _default_distance[class_id]);
    if (priv_nh.hasParam(prefix + "distance_variance"))
      priv_nh.getParam(prefix + "distance_variance", _distance_variance[class_id]);
    if (priv_nh.hasParam(prefix + "angle_variance"))
      priv_nh.getParam(prefix + "angle_variance", _angle_variance[class_id]);
    if (priv_nh.hasParam(prefix + "min_height"))
      priv_nh.getParam(prefix + "min_height", _min_height[class_id]);
    if (priv_nh.hasParam(prefix + "max_height"))
      priv_nh.getParam(prefix + "max_height", _max_height[class_id]);
    if (priv_nh.hasParam(prefix + "pending_support"))
      priv_nh.getParam(prefix + "pending_support", _pending_support[class_id]);
    if (priv_nh.hasParam(prefix + "pending_time"))
      priv_nh.getParam(prefix + "pending_time", _pending_time[class_id]);
    if (priv_nh.hasParam(prefix + "active_support"))
      priv_nh.getParam(prefix + "active_support", _active_support[class_id]);
    if (priv_nh.hasParam(prefix + "active_time"))
      priv_nh.getParam(prefix + "active_time", _active_time[class_id]);
    if (priv_nh.hasParam(prefix + "inactive_support"))
      priv_nh.getParam(prefix + "inactive_support", _inactive_support[class_id]);
    if (priv_nh.hasParam(prefix + "inactive_time"))
      priv_nh.getParam(prefix + "inactive_time", _inactive_time[class_id]);
    if (priv_nh.hasParam(prefix + "min_distance_between_objects"))
      priv_nh.getParam(prefix + "min_distance_between_objects", _min_distance_between_objects[class_id]);

    if (priv_nh.hasParam(prefix + "marker_color")) {
      XmlRpc::XmlRpcValue color;
      priv_nh.getParam(prefix + "marker_color", color);
      if (color.getType() == XmlRpc::XmlRpcValue::TypeArray && color.size() >= 3) {
        _marker_color[class_id].r = (double) color[0];
        _marker_color[class_id].g = (double) color[1];
        _marker_color[class_id].b = (double) color[2];
        if (color.size() > 3)
          _marker_color[class_id].a = (double) color[3];
        else
          _marker_color[class_id].a = 1.0;
      } else {
        ROS_ERROR_STREAM("Illegal value for param marker_color: " << color << ". Must be an array of size 3 or 4.");
      }
    }

    std::string distance_to_obstacle_service;

    if (priv_nh.hasParam(prefix + "distance_to_obstacle_service")) {
      priv_nh.getParam(prefix + "distance_to_obstacle_service", distance_to_obstacle_service);
    } else if (class_id.empty()) {

ROS_ERROR("In parameters.cpp  else if() { "); //3

      distance_to_obstacle_service = "hector_map_server/get_distance_to_obstacle";
    }

ROS_ERROR("Go from object_tracker.cpp  |  In parameters.cpp  |   Before _distance_to_obstacle_service..."); //4

// 这个service在hector_map_server.cpp

    if (!distance_to_obstacle_service.empty()) 
    {
      _distance_to_obstacle_service[class_id].reset(new ros::ServiceClient(ros::NodeHandle().serviceClient<hector_nav_msgs::GetDistanceToObstacle>(distance_to_obstacle_service)));

ROS_ERROR("In parameter.cpp  After getDistanceToObstacl e service__Client");

      bool project_objects = _project_objects.count(class_id) ? _project_objects.at(class_id) : _project_objects.at(std::string());

      if (project_objects && !_distance_to_obstacle_service[class_id]->waitForExistence(ros::Duration(5.0))) 
      {
        ROS_WARN_STREAM("project_objects is true, but GetDistanceToObstacle service is not (yet) available" << (!class_id.empty() ? "for class " + class_id : ""));
      } // end of if (project_objects && 
    } // end of if (!distance_to_obstacle_service.empty())

    if (priv_nh.hasParam(prefix + "get_normal_service")) {
      std::string get_normal_service;
      priv_nh.getParam(prefix + "get_normal_service", get_normal_service);
      _get_normal_service[class_id].reset(new ros::ServiceClient(ros::NodeHandle().serviceClient<hector_nav_msgs::GetNormal>(get_normal_service)));
    }

    _per_class_parameters_loaded.insert(class_id);

ROS_INFO("In parameters.cpp  leave parameter!!! ");
  }

} // namespace Parameters
} // namespace hector_object_tracker
