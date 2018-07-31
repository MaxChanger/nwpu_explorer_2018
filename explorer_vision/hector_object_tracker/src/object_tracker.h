
#ifndef OBJECT_TRACKER_OBJECT_TRACKER_H
#define OBJECT_TRACKER_OBJECT_TRACKER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <hector_object_tracker/types.h>
#include <hector_worldmodel_msgs/SetObjectState.h>
#include <hector_worldmodel_msgs/SetObjectName.h>
#include <hector_worldmodel_msgs/AddObject.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

#include <hector_marker_drawing/HectorDrawings.h>

#include "ObjectModel.h"

namespace hector_object_tracker {

class ObjectTracker {
public:
  ObjectTracker();
  virtual ~ObjectTracker();

protected:
  void sysCommandCb(const std_msgs::StringConstPtr &);
  void imagePerceptCb(const hector_worldmodel_msgs::ImagePerceptConstPtr &);
  void posePerceptCb(const hector_worldmodel_msgs::PosePerceptConstPtr &);
  void objectAgeingCb(const std_msgs::Float32ConstPtr &);

  void modelUpdateCb(const hector_worldmodel_msgs::ObjectModelConstPtr &);

  bool setObjectStateCb(hector_worldmodel_msgs::SetObjectState::Request& request, hector_worldmodel_msgs::SetObjectState::Response& response);
  bool setObjectNameCb(hector_worldmodel_msgs::SetObjectName::Request& request, hector_worldmodel_msgs::SetObjectName::Response& response);
  bool addObjectCb(hector_worldmodel_msgs::AddObject::Request& request, hector_worldmodel_msgs::AddObject::Response& response);
  bool getObjectModelCb(hector_worldmodel_msgs::GetObjectModel::Request& request, hector_worldmodel_msgs::GetObjectModel::Response& response);

  ObjectModel& getModel()             { return model; }
  const ObjectModel& getModel() const { return model; }

  ObjectModel getMergedModel();

  void publishModelEvent(const ros::TimerEvent&);
  void publishModel();

protected:
  bool transformPose(const geometry_msgs::Pose& from, geometry_msgs::Pose &to, std_msgs::Header &header, tf::StampedTransform *transform = 0);
  bool transformPose(const geometry_msgs::PoseWithCovariance& from, geometry_msgs::PoseWithCovariance &to, std_msgs::Header &header);
  bool mapToNextObstacle(const geometry_msgs::Pose& source, const std_msgs::Header &header, const ObjectInfo &info, geometry_msgs::Pose &mapped);

private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  ros::Subscriber imagePerceptSubscriber;
  ros::Subscriber posePerceptSubscriber;
  ros::Subscriber sysCommandSubscriber;
  ros::Subscriber objectAgeingSubscriber;

  struct NegativeUpdateInfo {
    std::string class_id;
    ObjectInfo::_support_type negative_support;
    ObjectInfo::_support_type min_support;
    float min_distance;
    float max_distance;
    float ignore_border_pixels;
    ros::Duration not_seen_duration;
    ros::Subscriber subscriber;
  };
  typedef boost::shared_ptr<NegativeUpdateInfo> NegativeUpdatePtr;
  std::vector<NegativeUpdatePtr> negativeUpdate;
  void negativeUpdateCallback(const sensor_msgs::CameraInfoConstPtr &, const NegativeUpdatePtr& info);

  ros::Publisher modelPublisher;
  ros::Publisher modelUpdatePublisher;
  ros::Publisher perceptPoseDebugPublisher;
  ros::Publisher objectPoseDebugPublisher;
  ros::Subscriber modelUpdateSubscriber;

  ros::ServiceServer setObjectState;
  ros::ServiceServer addObject;
  ros::ServiceServer getObjectModel;
  ros::ServiceServer setObjectName;

  ros::Timer publishTimer;

  tf::TransformListener tf;

  HectorDrawings drawings;

  ObjectModel model;
  typedef boost::shared_ptr<image_geometry::PinholeCameraModel> CameraModelPtr;
  std::map<std::string,CameraModelPtr> cameraModels;
  ObjectTracker::CameraModelPtr getCameraModel(const std::string& frame_id, const sensor_msgs::CameraInfo& camera_info);

  struct MergedModelInfo {
    std::string prefix;
    ObjectModel model;
    ros::Subscriber subscriber;
  };
  typedef boost::shared_ptr<MergedModelInfo> MergedModelPtr;
  std::vector<MergedModelPtr> merged_models;
  void mergeModelCallback(const hector_worldmodel_msgs::ObjectModelConstPtr &new_model, const MergedModelPtr& info);

  std::string _frame_id;
  std::string _worldmodel_ns;

  double _ageing_threshold;
  double _publish_interval;
};

} // namespace hector_object_tracker

#endif // OBJECT_TRACKER_OBJECT_TRACKER_H
