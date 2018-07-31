
#ifndef OBJECT_TRACKER_PARAMETERS_H
#define OBJECT_TRACKER_PARAMETERS_H

#include <string>
#include <map>
#include <std_msgs/ColorRGBA.h>
#include <ros/service_client.h>

#include <XmlRpcValue.h>

namespace hector_object_tracker {

  extern std::map<std::string, bool>   _project_objects;
  extern std::map<std::string, bool>   _with_orientation;
  extern std::map<std::string, double> _default_distance;
  extern std::map<std::string, double> _distance_variance;
  extern std::map<std::string, double> _angle_variance;
  extern std::map<std::string, double> _min_height;
  extern std::map<std::string, double> _max_height;
  extern std::map<std::string, double> _pending_support;
  extern std::map<std::string, double> _pending_time;
  extern std::map<std::string, double> _active_support;
  extern std::map<std::string, double> _active_time;
  extern std::map<std::string, double> _inactive_support;
  extern std::map<std::string, double> _inactive_time;
  extern std::map<std::string, double> _min_distance_between_objects;
  extern std::map<std::string, std_msgs::ColorRGBA> _marker_color;
  extern std::map<std::string, ros::ServiceClientPtr> _distance_to_obstacle_service;
  extern std::map<std::string, ros::ServiceClientPtr> _get_normal_service;

  typedef std::pair<ros::ServiceClientPtr, XmlRpc::XmlRpcValue> ServiceClientWithProperties;
  typedef std::vector<ServiceClientWithProperties> ServiceClientsWithProperties;
  extern std::map<std::string, ServiceClientsWithProperties> _percept_verification_services;
  extern std::map<std::string, ServiceClientsWithProperties> _object_verification_services;

  namespace Parameters {
    void load(const std::string& class_id = std::string());
  }

  template <typename T> static inline T& parameter(std::map<std::string, T>& p, const std::string& class_id = std::string()) {
    if (p.count(class_id)) return p.at(class_id);
    return p[std::string()];
  }

} // namespace hector_object_tracker

#endif // OBJECT_TRACKER_PARAMETERS_H
