
#include "object_tracker.h"

#include <hector_nav_msgs/GetDistanceToObstacle.h>
#include <hector_nav_msgs/GetNormal.h>
#include <hector_worldmodel_msgs/VerifyObject.h>
#include <hector_worldmodel_msgs/VerifyPercept.h>

#include <math.h>

#include "Object.h"
#include "parameters.h"

#include <boost/algorithm/string.hpp>

namespace hector_object_tracker {

ObjectTracker::ObjectTracker()
{
  ros::NodeHandle priv_nh("~");
  priv_nh.param("frame_id", _frame_id, std::string("map"));
  priv_nh.param("worldmodel_ns", _worldmodel_ns, std::string("worldmodel"));
  priv_nh.param("ageing_threshold", _ageing_threshold, 1.0);
  priv_nh.param("publish_interval", _publish_interval, 0.0);


  parameter(_project_objects)   = false;
  parameter(_with_orientation)  = false;
  parameter(_default_distance)  = 1.0;
  parameter(_distance_variance) = pow(1.0, 2);
  parameter(_angle_variance)    = pow(10.0 * M_PI / 180.0, 2);
  parameter(_min_height)        = -999.9;
  parameter(_max_height)        = 999.9;
  parameter(_pending_support)   = 0.0;
  parameter(_pending_time)      = 0.0;
  parameter(_active_support)    = 0.0;
  parameter(_active_time)       = 0.0;
  parameter(_inactive_support)  = 0.0;
  parameter(_inactive_time)     = 0.0;
  parameter(_min_distance_between_objects) = 0.0;

  std_msgs::ColorRGBA default_color;
  default_color.r = 0.8; default_color.a = 1.0;
  parameter(_marker_color)      = default_color;

//首先获得的是一个空的
//进入parameter的load()
ROS_ERROR("Before parameters::load()");    //2
  Parameters::load();

  ros::NodeHandle worldmodel(_worldmodel_ns);//in-3-4

ROS_ERROR("In object_tracker.cpp     In ObjectTracker.. 1"); //5

  imagePerceptSubscriber = worldmodel.subscribe("image_percept", 10, &ObjectTracker::imagePerceptCb, this);

  posePerceptSubscriber = worldmodel.subscribe("pose_percept", 10, &ObjectTracker::posePerceptCb, this); // pose_percept中有对object的状态
                                                                                                         // 的设置
  objectAgeingSubscriber = worldmodel.subscribe("object_ageing", 10, &ObjectTracker::objectAgeingCb, this);
// 这个也没有开 object_ageing

  modelPublisher = worldmodel.advertise<hector_worldmodel_msgs::ObjectModel>("objects", 10, false);
//发布当前扫到的全部的二维码的信息。都是相对于map的坐标

  modelUpdatePublisher = worldmodel.advertise<hector_worldmodel_msgs::Object>("object", 10, false);
  modelUpdateSubscriber = worldmodel.subscribe<hector_worldmodel_msgs::ObjectModel>("update", 10, &ObjectTracker::modelUpdateCb, this);
// 这个topic还没开  update


ROS_ERROR("In object_tracker.cpp  ...2"); //6

  Object::setNamespace(_worldmodel_ns);
  model.setFrameId(_frame_id);

  sysCommandSubscriber = nh.subscribe("/syscommand", 10, &ObjectTracker::sysCommandCb, this);
  perceptPoseDebugPublisher = priv_nh.advertise<geometry_msgs::PoseStamped>("percept/pose", 10, false);
  // 发布为 object_tracker/..  这个是在物理世界中，QR再camera_rgb_optical_frame坐标系的位置
  objectPoseDebugPublisher  = priv_nh.advertise<geometry_msgs::PoseStamped>("object/pose", 10, false);
   // 发布为 object_tracker/.. 这个是在地图中，QR在map坐标系的坐标

//根据stefan自己表示的hector基本不使用二维的标记，所以直接保存geotiff数据是不可能有标记的，他们一定经过了转化，既然这边发射的坐标，
//那么，一定在某一个中枢进行了融合。15/08/2015 18:49:23 
// 还少了一个node，是worldmodel_plugin中的


  XmlRpc::XmlRpcValue verification_services;
  if (priv_nh.getParam("verification_services", verification_services) && verification_services.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for(int i = 0; i < verification_services.size(); ++i) {
      XmlRpc::XmlRpcValue item = verification_services[i];
      if (!item.hasMember("service")) {
        ROS_ERROR("Verification service %d could not be intialized: no service name given", i);
        continue;
      }
      if (!item.hasMember("type")) {
        ROS_ERROR("Verification service %d could not be intialized: no service type given", i);
        continue;
      }

      ros::ServiceClientPtr client(new ros::ServiceClient);
      if (item["type"] == "object") {
        *client = nh.serviceClient<hector_worldmodel_msgs::VerifyObject>(item["service"]);
      } else if (item["type"] == "percept") {
        *client = nh.serviceClient<hector_worldmodel_msgs::VerifyPercept>(item["service"]);
      }

      if (!client || !client->isValid()) continue;
      if (!client->exists()) {
        if (!item.hasMember("required") || !item["required"]) {
          ROS_WARN("Verification service %s is not (yet) there...", client->getService().c_str());
        } else {
          ROS_WARN("Required verification service %s is not available... waiting...", client->getService().c_str());
          while(ros::ok() && !client->waitForExistence(ros::Duration(1.0)));
        }
      }

      std::string class_id;
      if (item.hasMember("class_id")) class_id = std::string(item["class_id"]);

      if (item["type"] == "object") {
        parameter(_object_verification_services, class_id).push_back(std::make_pair(client, item));
        if (class_id.empty()) {
          ROS_INFO("Using object verification service %s for all object classes", client->getService().c_str());
        } else {
          ROS_INFO("Using object verification service %s for objects of class %s", client->getService().c_str(), class_id.c_str());
        }
      } else if (item["type"] == "percept") {

ROS_ERROR("In object_tracker    else if (item['type'])");

        parameter(_percept_verification_services, class_id).push_back(std::make_pair(client, item));
        if (class_id.empty()) {
          ROS_INFO("Using percept verification service %s for all percept classes", client->getService().c_str());
        } else {
          ROS_INFO("Using percept verification service %s for percepts of class %s", client->getService().c_str(), class_id.c_str());
        } // end of else{
      }//end of' else if (item["type"] == "percept") {'


    }// end of for(int i = 0; i < verification_services.size(); ++i) {

  } // end of if (priv_nh.getParam("verification_services", verification_services) && verification_services.getType() == XmlRpc::XmlRpcValue::TypeArray) {

//  XmlRpc::XmlRpcValue get_normal_service;
//  if (priv_nh.getParam("get_normal_services", get_normal_service) && get_normal_service.getType() == XmlRpc::XmlRpcValue::TypeArray) {
//    for(int i = 0; i < get_normal_service.size(); ++i) {
//      XmlRpc::XmlRpcValue item = get_normal_service[i];
//      if (!item.hasMember("service")) {
//        ROS_ERROR("GetNormal service %d could not be intialized: no service name given", i);
//        continue;
//      }

//      ros::ServiceClientPtr client(new ros::ServiceClient);
//      *client = nh.serviceClient<hector_nav_msgs::GetNormal>(item["service"]);

//      if (!client || !client->isValid()) continue;
//      if (!client->exists()) {
//        if (!item.hasMember("required") || !item["required"]) {
//          ROS_WARN("GetNormal service %s is not (yet) there...", client->getService().c_str());
//        } else {
//          ROS_WARN("Required GetNormal service %s is not available... waiting...", client->getService().c_str());
//          while(ros::ok() && !client->waitForExistence(ros::Duration(1.0)));
//        }
//      }

//      if (item.hasMember("class_id")) {
//        parameter(_get_normal_service, item["class_id"]) = client;
//        ROS_INFO("Using GetNormal service %s for objects of class %s", client->getService().c_str(), std::string(item["class_id"]).c_str());
//      } else {
//        parameter(_get_normal_service) = client;
//        ROS_INFO("Using GetNormal service %s for all object classes", client->getService().c_str());
//      }
//    }
//  }

ROS_ERROR("In object_tracker.cpp       leave // | //");//6

  setObjectState = worldmodel.advertiseService("set_object_state", &ObjectTracker::setObjectStateCb, this);
  setObjectName  = worldmodel.advertiseService("set_object_name", &ObjectTracker::setObjectNameCb, this);

  addObject = worldmodel.advertiseService("add_object", &ObjectTracker::addObjectCb, this);
  getObjectModel = worldmodel.advertiseService("get_object_model", &ObjectTracker::getObjectModelCb, this);

  XmlRpc::XmlRpcValue merge;
  if (priv_nh.getParam("merge", merge) && merge.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for(int i = 0; i < merge.size(); ++i) {
      const MergedModelPtr& info = *merged_models.insert(merged_models.end(), boost::make_shared<MergedModelInfo>());
      ros::SubscribeOptions options = ros::SubscribeOptions::create<hector_worldmodel_msgs::ObjectModel>(std::string(), 10, boost::bind(&ObjectTracker::mergeModelCallback, this, _1, info), ros::VoidConstPtr(), 0);
      if (merge[i].getType() == XmlRpc::XmlRpcValue::TypeStruct && merge[i].hasMember("topic")) {
        options.topic = static_cast<std::string>(merge[i]["topic"]);
        if (merge[i].hasMember("prefix")) info->prefix = static_cast<std::string>(merge[i]["prefix"]);
      } else if (merge[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
        options.topic = static_cast<std::string>(merge[i]);
      }

      if (options.topic.empty()) {
        ROS_ERROR("Each entry in parameter merge must be either a string containing the topic to subscribe or a struct.");
        continue;
      }
      info->subscriber = nh.subscribe(options);

      ROS_INFO("Subscribed to object model %s.", options.topic.c_str());
    }
  }

  XmlRpc::XmlRpcValue negative_update;
  if (priv_nh.getParam("negative_update", negative_update) && negative_update.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for(int i = 0; i < negative_update.size(); ++i) {
      if (negative_update[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      const NegativeUpdatePtr& info = *negativeUpdate.insert(negativeUpdate.end(), boost::make_shared<NegativeUpdateInfo>());

      // default options
      info->negative_support = 0.0;
      info->min_support = 0.0;
      info->min_distance = 0.0;
      info->max_distance = 0.0;
      info->ignore_border_pixels = 0.0;
      info->not_seen_duration = ros::Duration(0.5);

      ros::SubscribeOptions options = ros::SubscribeOptions::create<sensor_msgs::CameraInfo>(std::string(), 10, boost::bind(&ObjectTracker::negativeUpdateCallback, this, _1, info), ros::VoidConstPtr(), 0);
      if (negative_update[i].hasMember("topic"))                options.topic = static_cast<std::string>(negative_update[i]["topic"]);
      if (negative_update[i].hasMember("class_id"))             info->class_id = static_cast<std::string>(negative_update[i]["class_id"]);
      if (negative_update[i].hasMember("negative_support"))     info->negative_support = static_cast<double>(negative_update[i]["negative_support"]);
      if (negative_update[i].hasMember("min_support"))          info->min_support = static_cast<double>(negative_update[i]["min_support"]);
      if (negative_update[i].hasMember("min_distance"))         info->min_distance = static_cast<double>(negative_update[i]["min_distance"]);
      if (negative_update[i].hasMember("max_distance"))         info->max_distance = static_cast<double>(negative_update[i]["max_distance"]);
      if (negative_update[i].hasMember("ignore_border_pixels")) info->ignore_border_pixels = static_cast<double>(negative_update[i]["ignore_border_pixels"]);
      if (negative_update[i].hasMember("not_seen_duration"))    info->not_seen_duration = ros::Duration(static_cast<double>(negative_update[i]["not_seen_duration"]));

      if (options.topic.empty()) {
        ROS_ERROR("Each entry in parameter negative_update must have a camera_info topic to subscribe to.");
        continue;
      }
      info->subscriber = nh.subscribe(options);
    }
  }

  if (_publish_interval > 0.0) {
    publishTimer = nh.createTimer(ros::Duration(_publish_interval), &ObjectTracker::publishModelEvent, this, false, true);
  }


ROS_ERROR("In object_tracker.cpp       out ObjectTracker....");//7

}

ObjectTracker::~ObjectTracker()
{}

void ObjectTracker::sysCommandCb(const std_msgs::StringConstPtr &sysCommand)
{
  if (sysCommand->data == "reset") {
    ROS_INFO("Resetting object model.");
    model.reset();
    drawings.reset();
    for(std::vector<MergedModelPtr>::const_iterator it =  merged_models.begin(); it != merged_models.end(); ++it) {
      (*it)->model.reset();
    }
  }
}



// 得到image_percept之后的统一处理，这边也有对当前扫到的标记的状态的设置
void ObjectTracker::imagePerceptCb(const hector_worldmodel_msgs::ImagePerceptConstPtr &percept)
{

ROS_ERROR("In object_tracker.cpp     In imagePerceptCb  ");

  hector_worldmodel_msgs::PosePerceptPtr posePercept(new hector_worldmodel_msgs::PosePercept);
  tf::Pose pose;


ROS_ERROR("In object_tracker.cpp, percept->info.class_id = [%s]  ",(percept->info.class_id).c_str());



  Parameters::load(percept->info.class_id);


  ROS_DEBUG("Incoming image percept with image coordinates [%f,%f] in frame %s", percept->x, percept->y, percept->header.frame_id.c_str());

  posePercept->header = percept->header;
  posePercept->info = percept->info;

  // retrieve distance information
  float distance = percept->distance > 0.0 ? percept->distance : parameter(_default_distance, percept->info.class_id);

  // retrieve camera model from either the cache or from CameraInfo given in the percept
  CameraModelPtr cameraModel = getCameraModel(percept->header.frame_id, percept->camera_info);


ROS_ERROR("In object_tracker.cpp    after  middle getCameraMOdel...mid     in imagePerceptCb ");
ROS_ERROR("->%s" ,percept->header.frame_id.c_str() );

ROS_ERROR("-->[%d]  [%d]",percept->x,percept->y);
/*
if(percept->y > 10000000)
{
    percept->y = 0;
}
*/
  // transform Point using the camera model

/**

这个下面的存在限定了好像只有带有深度信息的才ok， 的确！！！！

*/
  cv::Point2d rectified = cameraModel->rectifyPoint(cv::Point2d(percept->x, percept->y)); // !!!!

/*
terminate called after throwing an instance of 'image_geometry::Exception'
  what():  Cannot call rectifyPoint when distortion is unknown.

*/


  cv::Point3d direction_cv = cameraModel->projectPixelTo3dRay(rectified);
  tf::Point direction(direction_cv.x, direction_cv.y, direction_cv.z);
  direction.normalize();

 // pose.setOrigin(tf::Point(direction_cv.z, -direction_cv.x, -direction_cv.y).normalized() * distance);
 // tf::Quaternion direction(atan2(-direction_cv.x, direction_cv.z), atan2(direction_cv.y, sqrt(direction_cv.z*direction_cv.z + direction_cv.x*direction_cv.x)), 0.0);
  pose.setOrigin(tf::Point(direction_cv.x, direction_cv.y, direction_cv.z).normalized() * distance);
  {
    // set rotation of object so that the x-axis points in the direction of the object and y-axis is parallel to the camera's x-z-plane
    // Note: d is given in camera coordinates, while the object's x-axis should point away from the camera.
    const tf::Point &d(direction); // for readability

    if (d.y() >= 0.999) {
      pose.setBasis(tf::Matrix3x3( 0., -1.,  0.,
                                   1.,  0.,  0.,
                                   0.,  0.,  1. ));
    } else if (d.y() <= -0.999) {
      pose.setBasis(tf::Matrix3x3( 0., -1.,  0.,
                                  -1.,  0.,  0.,
                                   0.,  0., -1.));
    } else {
      double c = 1./sqrt(1. - d.y()*d.y());
//      pose.setBasis(tf::Matrix3x3( c*d.z(), -c*d.x()*d.y(), d.x(),
//                                         0, 1./c,           d.y(),
//                                  -c*d.x(), -c*d.y()*d.z(), d.z()));
      pose.setBasis(tf::Matrix3x3(d.x(), -c*d.z(), c*d.x()*d.y(),
                                  d.y(),        0,         -1./c,
                                  d.z(),  c*d.x(), c*d.y()*d.z() ));
    }
  }


ROS_ERROR("++++++=====+++++ imagePerceptCb=======MID");

/*
  ROS_DEBUG("--> Rectified image coordinates: [%f,%f]", rectified.x, rectified.y);
  ROS_DEBUG("--> Projected 3D ray (OpenCV):   [%f,%f,%f]", direction_cv.x, direction_cv.y, direction_cv.z);
  ROS_DEBUG("--> Projected 3D ray (tf):       [%f,%f,%f]", pose.getOrigin().x(), pose.getOrigin().y(),pose.getOrigin().z());
*/

//  下面又出现了getdistancetoObstacle的service如果没有这个service，那么就不能执行下面
  if (percept->distance == 0.0 && parameter(_project_objects, percept->info.class_id)) {
    hector_nav_msgs::GetDistanceToObstacle::Request request;
    hector_nav_msgs::GetDistanceToObstacle::Response response;

ROS_ERROR("In object_tracker.cpp  IM   in");

    // project image percept to the next obstacle
    request.point.header = percept->header;
    tf::pointTFToMsg(pose.getOrigin(), request.point.point);
    ros::ServiceClientPtr client = parameter(_distance_to_obstacle_service, percept->info.class_id);

ROS_ERROR("In object_tracker.cpp  .distance service: [%s]",percept->info.class_id.c_str());

//////虚拟的QR位置

//distance = std::max(5.0f, 0.0f);
//pose.setOrigin(pose.getOrigin().normalized() * distance);


    if (client && client->call(request, response)) {
      if (response.distance > 0.0) {
        // distance = std::max(response.distance - 0.1f, 0.0f);
        distance = std::max(response.distance, 0.0f);
        pose.setOrigin(pose.getOrigin().normalized() * distance);
        ROS_DEBUG("Projected percept to a distance of %.1f m", distance);
        ROS_ERROR("client & client->call [OK]");
      } else {
        ROS_WARN("Ignoring percept due to unknown or infinite distance: service %s returned %f", client->getService().c_str(), response.distance);
        //ROS_ERROR("ERROR!! in object_tracker.cpp response.distance <= 0.0");
        return;
      }
    } else {
      ROS_ERROR("ERROR!! in object_tracker.cpp  client && client->call [false]");
//说明这边，client && client->call(request , response) == false
      ROS_WARN("Ignoring percept due to unknown or infinite distance: service is not available");
      return;
    }


  } // end of   if (percept->distance == 0.0 && parameter(_project_objects, percept->info.class_id)) {


// 问题在上面，那边就已经返回了
ROS_ERROR("RETURN????");

  // set variance
  Eigen::Matrix3f covariance(Eigen::Matrix3f::Zero());
  covariance(0,0) = std::max(distance*distance, 1.0f) * tan(parameter(_angle_variance, percept->info.class_id));
  covariance(1,1) = covariance(0,0);
  covariance(2,2) = parameter(_distance_variance, percept->info.class_id);

  // rotate covariance matrix depending on the position in the image
  Eigen::Matrix3f rotation_camera_object;
  rotation_camera_object << pose.getBasis()[0][0], pose.getBasis()[0][1], pose.getBasis()[0][2],
                            pose.getBasis()[1][0], pose.getBasis()[1][1], pose.getBasis()[1][2],
                            pose.getBasis()[2][0], pose.getBasis()[2][1], pose.getBasis()[2][2];
  covariance = rotation_camera_object * covariance * rotation_camera_object.transpose();

  // fill posePercept
  tf::poseTFToMsg(pose, posePercept->pose.pose);
  // tf::quaternionTFToMsg(direction, posePercept->pose.pose.orientation);
  posePercept->pose.covariance[0]  = covariance(0,0);
  posePercept->pose.covariance[1]  = covariance(0,1);
  posePercept->pose.covariance[2]  = covariance(0,2);
  posePercept->pose.covariance[6]  = covariance(1,0);
  posePercept->pose.covariance[7]  = covariance(1,1);
  posePercept->pose.covariance[8]  = covariance(1,2);
  posePercept->pose.covariance[12] = covariance(2,0);
  posePercept->pose.covariance[13] = covariance(2,1);
  posePercept->pose.covariance[14] = covariance(2,2);

  // forward to posePercept callback
  posePerceptCb(posePercept); // 然后直接是调用下面的函数


ROS_ERROR("In object_tracker.cpp       Out imagePerceptCb");
}




// 在这个函数中，可以是更根据id的内容进行对象的判断，然后，再通过，当为victim时，直接完全阻断，来实现吗？







void ObjectTracker::posePerceptCb(const hector_worldmodel_msgs::PosePerceptConstPtr &percept)
{

ROS_ERROR("In object_tracker.cpp     in posePerceptCb  ");

ROS_ERROR("In object_tracker.cpp     percept->info.class_id = [%s]",(percept->info.class_id).c_str() );

  Parameters::load(percept->info.class_id);

  // publish pose in source frame for debugging purposes
  if (perceptPoseDebugPublisher.getNumSubscribers() > 0 && percept->info.class_id!="qrcode") {
ROS_ERROR("In object_tracker.cpp      if (perceptPoseDebugPublisher.getNumSubscribers() > 0) ");
    geometry_msgs::PoseStamped pose;
    pose.pose = percept->pose.pose;
    pose.header = percept->header;
    perceptPoseDebugPublisher.publish(pose);
  }

  // call percept verification
  float support_added_by_percept_verification = 0.0;

  ServiceClientsWithProperties &percept_verification_services = parameter(_percept_verification_services, percept->info.class_id);

  if (!percept_verification_services.empty()) {
    hector_worldmodel_msgs::VerifyPercept::Request request;
    hector_worldmodel_msgs::VerifyPercept::Response response;

    request.percept = *percept;

    for(ServiceClientsWithProperties::iterator it = percept_verification_services.begin(); it != percept_verification_services.end(); ++it) {
      if (!(it->first) || !(it->first->isValid())) continue;

      if (it->second.hasMember("ignore") && it->second["ignore"]) {
        ROS_DEBUG("Calling service %s for percept of class '%s'', but ignoring its answer...", it->first->getService().c_str(), percept->info.class_id.c_str());
        it->first->call(request, response);

      } else if (it->first->call(request, response)) {
        if (response.response == response.DISCARD) {
          ROS_DEBUG("Discarded percept of class '%s' due to DISCARD message from service %s", percept->info.class_id.c_str(), it->first->getService().c_str());
          return;
        }
        if (response.response == response.CONFIRM) {
          ROS_DEBUG("We got a CONFIRMation for percept of class '%s' from service %s!", percept->info.class_id.c_str(), it->first->getService().c_str());
          support_added_by_percept_verification = 100.0;
        }
        if (response.response == response.UNKNOWN) {
          ROS_DEBUG("Verification service %s cannot help us with percept of class %s at the moment :-(", it->first->getService().c_str(), percept->info.class_id.c_str());
        }
      } else if (it->second.hasMember("required") && it->second["required"]) {
        ROS_DEBUG("Discarded percept of class '%s' as required service %s is not available", percept->info.class_id.c_str(), it->first->getService().c_str());
        return;
      }
    }
  }

  // convert pose in tf
  tf::Pose pose;
  tf::poseMsgToTF(percept->pose.pose, pose);

  // retrieve distance information
//  float distance = pose.getOrigin().length();
//  if (parameter(_project_objects, percept->info.class_id)) {
//    hector_nav_msgs::GetDistanceToObstacle::Request request;
//    hector_nav_msgs::GetDistanceToObstacle::Response response;

//    // project image percept to the next obstacle
//    request.point.header = percept->header;
//    tf::pointTFToMsg(pose.getOrigin(), request.point.point);
//    if (parameter(_distance_to_obstacle_service, percept->info.class_id).call(request, response) && response.distance > 0.0) {
//      // distance = std::max(response.distance - 0.1f, 0.0f);
//      distance = std::max(response.distance, 0.0f);
//      pose.setOrigin(pose.getOrigin().normalized() * distance);
//      ROS_DEBUG("Projected percept to a distance of %.1f m", distance);
//    } else {
//      ROS_DEBUG("Ignoring percept due to unknown or infinite distance");
//      return;
//    }
//  }

  // extract variance matrix
  Eigen::Matrix<float,6,6> temp;
  for(unsigned int i = 0; i < 36; ++i) temp(i) = percept->pose.covariance[i];
  Eigen::Matrix3f covariance = temp.block<3,3>(0,0);

  // if no variance is given, set variance to default
  if (covariance.isZero()) {
    covariance(0,0) = parameter(_distance_variance, percept->info.class_id);
    covariance(1,1) = parameter(_distance_variance, percept->info.class_id);
    covariance(2,2) = parameter(_distance_variance, percept->info.class_id);
  }

  // project percept coordinates to map frame


//在上面设置了虚拟的QR之后，运行到这边，出现问题！！！！=============================   14/08/2015 01:42:00 
// 设置虚拟的Qr距离之后，可以在图上看到 QR标记点了， 的确是TF的问题
// map_server  那边也有mapCallback，那这边服务相对就ok

  tf::StampedTransform cameraTransform;

  if (!_frame_id.empty() && tf.resolve(percept->header.frame_id) != tf.resolve(_frame_id)) {
    ROS_DEBUG("Transforming percept from %s frame to %s frame", percept->header.frame_id.c_str(), _frame_id.c_str());

    // retrieve camera transformation from tf
    try {
      tf.waitForTransform(_frame_id, percept->header.frame_id, percept->header.stamp, ros::Duration(1.0));
      tf.lookupTransform(_frame_id, percept->header.frame_id, percept->header.stamp, cameraTransform);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());


ROS_ERROR("frame_problem?"); // 的确是TF 的问题

      return;
    }

    pose = cameraTransform * pose;

    // rotate covariance matrix to map coordinates
    Eigen::Matrix3f rotation_map_camera;
    rotation_map_camera << cameraTransform.getBasis()[0][0], cameraTransform.getBasis()[0][1], cameraTransform.getBasis()[0][2],
                           cameraTransform.getBasis()[1][0], cameraTransform.getBasis()[1][1], cameraTransform.getBasis()[1][2],
                           cameraTransform.getBasis()[2][0], cameraTransform.getBasis()[2][1], cameraTransform.getBasis()[2][2];
    covariance = rotation_map_camera * covariance * rotation_map_camera.transpose();
  }

  // check height
  float relative_height = pose.getOrigin().z() - cameraTransform.getOrigin().z();
  if (relative_height < parameter(_min_height, percept->info.class_id) || relative_height > parameter(_max_height, percept->info.class_id)) {
    ROS_INFO("Discarding %s percept with height %f", percept->info.class_id.c_str(), relative_height);
    return;
  }

  // get object orientation from normal to the wall
  ros::ServiceClientPtr get_normal_client = parameter(_get_normal_service, percept->info.class_id);
  if (get_normal_client) {
    // Get normal at object's position
    hector_nav_msgs::GetNormal get_normal;
    get_normal.request.point.point.x = pose.getOrigin().x();
    get_normal.request.point.point.y = pose.getOrigin().y();
    get_normal.request.point.point.z = pose.getOrigin().z();
    get_normal.request.point.header.frame_id = "map";
    get_normal.request.point.header.stamp = percept->header.stamp;

    if (get_normal_client->call(get_normal.request, get_normal.response)) {
      tf::Vector3 normal(get_normal.response.normal.x, get_normal.response.normal.y, get_normal.response.normal.z);
      normal.normalize(); // we don't trust the server :-)

      // invert normal if it points away from the percept's x-axis
      if (pose.getBasis().getRow(0) * normal < 0) {
        normal = -normal;
      }

      {
        // set rotation of object so that the x-axis points in the direction of the normal and y-axis is parallel to the x-y-plane
        const tf::Point &n(normal); // for readability
        if (n.z() >= 0.999) {
          pose.setBasis(tf::Matrix3x3(0, 0, -1., 0, 1., 0, 1., 0, 0));
        } else if (n.z() <= -0.999) {
          pose.setBasis(tf::Matrix3x3(0, 0, 1., 0, 1., 0, -1., 0, 0));
        } else {
          double c = 1./sqrt(1. - n.z()*n.z());
          pose.setBasis(tf::Matrix3x3(n.x(), -c*n.y(), -c*n.x()*n.z(),
                                      n.y(),  c*n.x(), -c*n.y()*n.z(),
                                      n.z(),        0, 1./c));
        }
      }

      tfScalar r,p,y;
      pose.getBasis().getRPY(r, p, y);
      ROS_DEBUG("Object orientation was updated from wall normals: %f", y * 180./M_PI);

    } else {
      tfScalar r,p,y;
      pose.getBasis().getRPY(r, p, y);
      ROS_DEBUG("View angle was used as object orientation: %f", y * 180./M_PI);
    }
  }

  // fix height (assume camera is always at 0.475m)
  // pose.setOrigin(tf::Point(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() - cameraTransform.getOrigin().z() + 0.475f));

  // calculate observation support
  float support = 0.0;
  if (!percept->info.object_id.empty()) {
    support = percept->info.object_support;
  } else if (!percept->info.class_id.empty()) {
    support = percept->info.class_support + support_added_by_percept_verification;
  }

  if (support == 0.0) {
    ROS_WARN("Ignoring percept with support == 0.0");
    return;
  }

  // lock model
  model.lock();

  // find correspondence
  ObjectPtr object;
  if (percept->info.object_id.empty()) {
    model.getBestCorrespondence(object, pose, covariance, percept->info.class_id, percept->info.name, 1.0f);
  } else {
    object = model.getObject(percept->info.object_id);
  }

  if (object && object->getState() < 0) {
    ROS_DEBUG("Percept was associated to object %s, which has a fixed state", object->getObjectId().c_str());
    model.unlock();
    return;
  }

  // create new object
  //注意这里我终于理解了，model对象的区分依据就是object_id,这也就解释了为什么标记受害者只能标记一个，妈也，我代码写完了才看到，算了，写了代码也更方便以后修改相关功能
  if (!object) {
    object = model.add(percept->info.class_id, percept->info.object_id);

    object->setPose(pose);
    object->setCovariance(covariance);
    object->setSupport(support);

    ROS_INFO("Found new object %s of class %s at (%f,%f)!", object->getObjectId().c_str(), object->getClassId().c_str(), pose.getOrigin().getX(), pose.getOrigin().getY());

  // or update existing object
  } else if (support > 0.0) {
    //object->update(pose, covariance, support);
    object->intersect(pose, covariance, support);

  // or simply decrease support
  } else {
    object->addSupport(support);
  }

  // update object state
  if ((object->getState() == ObjectState::UNKNOWN || object->getState() == ObjectState::INACTIVE) &&  parameter(_pending_support, percept->info.class_id) > 0) {
    if (object->getSupport() >= parameter(_pending_support, percept->info.class_id) && (percept->header.stamp - object->getHeader().stamp).toSec() >= parameter(_pending_time, percept->info.class_id)) {
      object->setState(ObjectState::PENDING);
    }
  }
  if (object->getState() == ObjectState::PENDING &&  parameter(_active_support, percept->info.class_id) > 0) {
    if (object->getSupport() >= parameter(_active_support, percept->info.class_id) && (percept->header.stamp - object->getHeader().stamp).toSec() >= parameter(_active_time, percept->info.class_id)) {
      object->setState(ObjectState::ACTIVE);
    }
  }

  // update object header
  std_msgs::Header header;
  header.stamp    = percept->header.stamp;
  header.frame_id = _frame_id;
  object->setHeader(header);

  // update object name
  if (!percept->info.name.empty()) object->setName(percept->info.name);

  // unlock model
  model.unlock();

  // call object verification
  ServiceClientsWithProperties &object_verification_services = parameter(_object_verification_services, percept->info.class_id);
  if (!object_verification_services.empty()) {
    hector_worldmodel_msgs::VerifyObject::Request request;
    hector_worldmodel_msgs::VerifyObject::Response response;

    object->getMessage(request.object);

    for(ServiceClientsWithProperties::iterator it = object_verification_services.begin(); it != object_verification_services.end(); ++it) {
      if (it->second.hasMember("ignore") && it->second["ignore"]) {
        ROS_DEBUG("Calling service %s for object %s, but ignoring its answer...", it->first->getService().c_str(), object->getObjectId().c_str());
        it->first->call(request, response);

      } else if (it->first->call(request, response)) {
        if (response.response == response.DISCARD) {
          ROS_DEBUG("Discarded object %s due to DISCARD message from service %s", object->getObjectId().c_str(), it->first->getService().c_str());
          object->setState(ObjectState::DISCARDED);
        }
        if (response.response == response.CONFIRM) {
          ROS_DEBUG("We got a CONFIRMation for object %s from service %s!", object->getObjectId().c_str(), it->first->getService().c_str());
          object->addSupport(100.0);
        }
        if (response.response == response.UNKNOWN) {
          ROS_DEBUG("Verification service %s cannot help us with object %s at the moment :-(", it->first->getService().c_str(), object->getObjectId().c_str());
        }
      } else if (it->second.hasMember("required") && it->second["required"]) {
        ROS_DEBUG("Discarded object %s as required service %s is not available", object->getObjectId().c_str(), it->first->getService().c_str());
        object->setState(ObjectState::DISCARDED);
      }
    }
  }

  // publish pose in target frame for debugging purposes
  if (objectPoseDebugPublisher.getNumSubscribers() > 0&&percept->info.class_id=="victim") {
    geometry_msgs::PoseStamped pose;
    object->getPose(pose.pose);
    pose.header = object->getHeader();
    objectPoseDebugPublisher.publish(pose);
  }

  modelUpdatePublisher.publish(object->getMessage());
  publishModel();


ROS_ERROR("In object_tracker.cpp     Out PosePerceptCb   ");

}

ObjectTracker::CameraModelPtr ObjectTracker::getCameraModel(const std::string& frame_id, const sensor_msgs::CameraInfo& camera_info) {
  // retrieve camera model from either the cache or from CameraInfo given in the percept
ROS_ERROR("In object_tracker.cpp  getCameraModel!!!");

  CameraModelPtr cameraModel;

ROS_ERROR("2015`9`18=> [%s]", frame_id.c_str());
ROS_ERROR("=== [%d]",cameraModels.count(frame_id));

  if (cameraModels.count(frame_id) == 0) {

ROS_ERROR("111");

    cameraModel.reset(new image_geometry::PinholeCameraModel());
    if (!cameraModel->fromCameraInfo(camera_info)) {

ROS_ERROR("222");

      ROS_ERROR("Could not initialize camera model from CameraInfo given in the percept");
      return CameraModelPtr();
    }
    cameraModels[frame_id] = cameraModel;
ROS_ERROR("333");

  } else {

ROS_ERROR("444");

    cameraModel = cameraModels[frame_id];
  }

ROS_ERROR("In object_tracker.cpp     Out getCameraModel  ");

  return cameraModel;
}

void ObjectTracker::objectAgeingCb(const std_msgs::Float32ConstPtr &ageing) {
  ROS_DEBUG("ageing of all objects by %f", ageing->data);

  // lock model
  model.lock();

  ObjectList objects = model.getObjects();

  for(ObjectModel::iterator it = objects.begin(); it != objects.end();) {
    ObjectPtr object = *it;

    // update support
    object->setSupport(object->getSupport() - ageing->data);

    // remove the object if the support is to low
    if (object->getSupport() < _ageing_threshold) {
      ROS_INFO("remove object %s with support %f", object->getObjectId().c_str(), object->getSupport());
      it = objects.erase(it);
      model.remove(object);
    } else {
      it++;
    }
  }

  // unlock model
  model.unlock();
  publishModel();


ROS_ERROR("In object_tracker.cpp      objectAgeingCb");
}

void ObjectTracker::modelUpdateCb(const hector_worldmodel_msgs::ObjectModelConstPtr &update)
{

  for(hector_worldmodel_msgs::ObjectModel::_objects_type::const_iterator it = update->objects.begin(); it != update->objects.end(); ++it)
  {
    hector_worldmodel_msgs::AddObject object;
    object.request.map_to_next_obstacle = false;
    object.request.object = *it;
    if (!addObjectCb(object.request, object.response)) {
      ROS_WARN("Could not update object %s", it->info.object_id.c_str());

ROS_ERROR(" In object_tracker.cpp   755 modelUpdateCb");

    }
  }


ROS_ERROR("In object_tracker.cpp     modelUpdateCb ");
}

bool ObjectTracker::setObjectStateCb(hector_worldmodel_msgs::SetObjectState::Request& request, hector_worldmodel_msgs::SetObjectState::Response& response) {

ROS_ERROR("In object_tracker.cpp     setObjectStateCb  ");

  model.lock();

  ObjectPtr object = model.getObject(request.object_id);
  if (!object) {
    model.unlock();
    return false;
  }

  object->setState(request.new_state.state);
  modelUpdatePublisher.publish(object->getMessage());

  // check minimum distance between objects and discard objects which are closer
  double min_distance_between_objects = parameter(_min_distance_between_objects, object->getClassId());
  if ((min_distance_between_objects > 0.0) && (request.new_state.state == hector_worldmodel_msgs::ObjectState::CONFIRMED)) {
    for(ObjectModel::iterator it = model.begin(); it != model.end(); ++it) {
      ObjectPtr other = *it;
      if (other == object) continue;
      if (other->getClassId() != object->getClassId()) continue;

      if (other->getDistance(*object) < min_distance_between_objects) {
        other->setState(hector_worldmodel_msgs::ObjectState::DISCARDED);
        modelUpdatePublisher.publish(other->getMessage());
      }
    }
  }

  model.unlock();
  publishModel();
  return true;
}

bool ObjectTracker::setObjectNameCb(hector_worldmodel_msgs::SetObjectName::Request& request, hector_worldmodel_msgs::SetObjectName::Response& response) {

ROS_ERROR("In object_tracker.cpp      setObjectNameCb ");

  model.lock();

  ObjectPtr object = model.getObject(request.object_id);
  if (!object) {
    model.unlock();
    return false;
  }

  object->setName(request.name);
  modelUpdatePublisher.publish(object->getMessage());

  model.unlock();
  publishModel();
  return true;
}

bool ObjectTracker::addObjectCb(hector_worldmodel_msgs::AddObject::Request& request, hector_worldmodel_msgs::AddObject::Response& response) {

ROS_ERROR("In object_tracker.cpp       addObjectCb ");

  ObjectPtr object;
  bool newObject = false;

  // check if object already exist
  if (!request.object.info.object_id.empty()) {
    ROS_INFO("add_object service called for known %s object %s in frame %s", request.object.info.class_id.c_str(), request.object.info.object_id.c_str(), request.object.header.frame_id.c_str());
    object = model.getObject(request.object.info.object_id);
  } else {
    ROS_INFO("add_object service called for new %s object in frame %s", request.object.info.class_id.c_str(), request.object.header.frame_id.c_str());
  }

  // create a new object if object does not exist
  if (!object) {
    object.reset(new Object(request.object.info.class_id, request.object.info.object_id));
    newObject = true;
  }

  std_msgs::Header header = request.object.header;
  if (header.stamp.isZero()) header.stamp = ros::Time::now();

  geometry_msgs::PoseWithCovariance pose;
  if (request.map_to_next_obstacle) {
    pose.covariance = request.object.pose.covariance;
    if (!mapToNextObstacle(request.object.pose.pose, header, request.object.info, pose.pose)) {
      return false;
    }
  } else {
    pose = request.object.pose;
  }

  // if no variance is given, set variance to default
  if (pose.covariance[0] == 0  && pose.covariance[7] == 0  && pose.covariance[14] == 0 &&
      pose.covariance[21] == 0 && pose.covariance[28] == 0 && pose.covariance[35] == 0) {
    pose.covariance.assign(0.0);
    pose.covariance[0] = 1.0;
    pose.covariance[7] = 1.0;
    pose.covariance[14] = 1.0;
  }

  if (!transformPose(pose, pose, header)) return false;

  model.lock();

  object->setHeader(header);
  object->setPose(pose);
  object->setState(request.object.state.state);
  object->setSupport(request.object.info.support);

  if (newObject) model.add(object);
  object->getMessage(response.object);
  modelUpdatePublisher.publish(response.object);

  model.unlock();

  publishModel();
  return true;
}

bool ObjectTracker::getObjectModelCb(hector_worldmodel_msgs::GetObjectModel::Request& request, hector_worldmodel_msgs::GetObjectModel::Response& response) {

ROS_ERROR("In object_tracker.cpp      getObjectModelCb");

  getMergedModel().getMessage(response.model);
  return true;
}

void ObjectTracker::mergeModelCallback(const hector_worldmodel_msgs::ObjectModelConstPtr &new_model, const MergedModelPtr& info)
{

ROS_ERROR("In object_tracker.cpp      mergeModelCallbacck ");

  info->model = *new_model;
  publishModel();
}

void ObjectTracker::negativeUpdateCallback(const sensor_msgs::CameraInfoConstPtr &camera_info, const NegativeUpdatePtr& info)
{

ROS_ERROR("In object_tracker.cpp      negativeUpdateCallback");


  model.lock();

  // retrieve camera model from either the cache or from CameraInfo given in the percept
  CameraModelPtr cameraModel = getCameraModel(camera_info->header.frame_id, *camera_info);

  // get camera transform
  tf::StampedTransform cameraInverseTransform;
  try {
    tf.waitForTransform(camera_info->header.frame_id, _frame_id, camera_info->header.stamp, ros::Duration(1.0));
    tf.lookupTransform(camera_info->header.frame_id, _frame_id, camera_info->header.stamp, cameraInverseTransform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  ROS_DEBUG_NAMED("negative_update", "Doing negative update for frame_id %s", camera_info->header.frame_id.c_str());

  // iterate through objects
  for(ObjectModel::iterator it = model.begin(); it != model.end(); ++it) {
    const ObjectPtr& object = *it;

    // do not update objects with state < 0
    if (object->getState() < 0) continue;

    // check class_id
    if (!info->class_id.empty() && info->class_id != object->getClassId()) {
      ROS_DEBUG_NAMED("negative_update", "%s: wrong class_id %s", object->getObjectId().c_str(), object->getClassId().c_str());
      continue;
    }

    // check last seen stamp
    if (object->getStamp() >= camera_info->header.stamp - info->not_seen_duration) {
      ROS_DEBUG_NAMED("negative_update", "%s: seen %f seconds ago", object->getObjectId().c_str(), (camera_info->header.stamp - object->getStamp()).toSec());
      continue;
    }

    // transform object pose to camera coordinates
    tf::Pose pose;
    object->getPose(pose);
    pose = cameraInverseTransform * pose;
    cv::Point3d xyz(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());

    // check distance
    float distance = pose.getOrigin().length();
    if (distance < info->min_distance || (info->max_distance > 0.0 && distance > info->max_distance)) {
      ROS_DEBUG_NAMED("negative_update", "%s: wrong distance: %f", object->getObjectId().c_str(), distance);
      continue;
    }

    // get image point
    cv::Point2d point = cameraModel->project3dToPixel(xyz);

    // check if object is within field of view
    if (!(point.x > 0 + info->ignore_border_pixels &&
          point.x < camera_info->width - info->ignore_border_pixels &&
          point.y > 0 + info->ignore_border_pixels &&
          point.y < camera_info->height - info->ignore_border_pixels)) {
      ROS_DEBUG_NAMED("negative_update", "%s: not within field of view (image coordinates (%f,%f))", object->getObjectId().c_str(), point.x, point.y);
      continue;
    }

    // ==> do negative update
    ROS_DEBUG("Doing negative update of %s. Should be at image coordinates (%f,%f).", object->getObjectId().c_str(), point.x, point.y);
    object->addSupport(-info->negative_support);
    if (object->getSupport() < info->min_support) object->setSupport(info->min_support);
    if (object->getSupport() <= parameter(_inactive_support, info->class_id)) {
      if (object->getState() == ObjectState::PENDING || object->getState() == ObjectState::ACTIVE)
        object->setState(ObjectState::INACTIVE);
    }

    // publish object update
    modelUpdatePublisher.publish(object->getMessage());
  }

  model.unlock();
  // publishModel();
}

bool ObjectTracker::mapToNextObstacle(const geometry_msgs::Pose& source, const std_msgs::Header &header, const ObjectInfo &info, geometry_msgs::Pose &mapped) {

ROS_ERROR("In object_tracker.cpp     mapToNextObstacle ");


  Parameters::load(info.class_id);
  ros::ServiceClientPtr client = parameter(_distance_to_obstacle_service, info.class_id);

  if (!client || !client->exists()) {
    ROS_ERROR("Could not map object to next obstacle as the distance service %s is not available", client->getService().c_str());
    return false;
  }

  // retrieve distance information
  float distance = parameter(_default_distance, info.class_id);
  hector_nav_msgs::GetDistanceToObstacle::Request request;
  hector_nav_msgs::GetDistanceToObstacle::Response response;

  // project image percept to the next obstacle
  request.point.header = header;
  request.point.point = source.position;
  // tf::pointTFToMsg(cameraTransform.getOrigin(), request.pose.pose.position);
  // tf::Quaternion direction_quaternion = tf::Quaternion(atan(direction.y/direction.x), atan(direction.z/direction.x), 0.0);
  // direction_quaternion *= cameraTransform.getRotation();
  // tf::quaternionTFToMsg(direction_quaternion, request.pose.pose.orientation);
  if (client->call(request, response) && response.distance > 0.0) {
    // distance = std::max(response.distance - 0.1f, 0.0f);
    distance = std::max(response.distance, 0.0f);
  } else {
    ROS_DEBUG("Could not map object to next obstacle due to unknown or infinite distance");
    return false;
  }

  tf::Pose sourceTF;
  tf::poseMsgToTF(source, sourceTF);
  sourceTF.setOrigin(sourceTF.getOrigin().normalized() * distance);
  tf::poseTFToMsg(sourceTF, mapped);

  return true;
}

bool ObjectTracker::transformPose(const geometry_msgs::Pose& from, geometry_msgs::Pose &to, std_msgs::Header &header, tf::StampedTransform *transform_ptr) {

ROS_ERROR("In object_tracker.cpp     transformPose 1");

  // retrieve transformation from tf
  tf::StampedTransform transform;
  try {
    tf.waitForTransform(_frame_id, header.frame_id, header.stamp, ros::Duration(1.0));
    tf.lookupTransform(_frame_id, header.frame_id, header.stamp, transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  tf::Pose tfPose;
  tf::poseMsgToTF(from, tfPose);
  tfPose = transform * tfPose;
  tf::poseTFToMsg(tfPose, to);

  header.frame_id = _frame_id;
  if (transform_ptr) *transform_ptr = transform;

  return true;
}

bool ObjectTracker::transformPose(const geometry_msgs::PoseWithCovariance& from, geometry_msgs::PoseWithCovariance &to, std_msgs::Header &header) {

ROS_ERROR("In object_tracker.cpp     transformPose 2");

  tf::StampedTransform transform;

  if (!transformPose(from.pose, to.pose, header, &transform)) return false;

  // TODO
  // rotate covariance matrix

  return true;
}

ObjectModel ObjectTracker::getMergedModel()
{
ROS_ERROR("IN objectTracker  getMergedModel");//11

  if (merged_models.size() == 0) return model;

ROS_ERROR("In objectTracker  After getMergedModel");//no

  ROS_DEBUG("Merging object models from %lu sources.", merged_models.size());

  // merge with other models from merged_models
  ObjectModel merged(model);
  for(std::vector<MergedModelPtr>::iterator it = merged_models.begin(); it != merged_models.end(); ++it)
  {
    merged.mergeWith((*it)->model, tf, (*it)->prefix);
  }

  return merged;
}

void ObjectTracker::publishModelEvent(const ros::TimerEvent&) {

ROS_ERROR("In object_tracker.cpp      publishModelEvent ");//9

  publishModel();
}

void ObjectTracker::publishModel() {

ROS_ERROR("In objectTracker.cpp publishModel"); ////////////////10

  ObjectModel merged_model = getMergedModel(); // into and direct return; /// important!!!!!! 
// as merged_model is empty()!

  // Publish all model data on topic /objects
  modelPublisher.publish(merged_model.getMessage());

  // Visualize objects and covariance in rviz
  visualization_msgs::MarkerArray markers;
  merged_model.getVisualization(markers);
//==++++++++++++++++++++++++++++++++++++++
//下面这个是产生一个原点周围的红色大区域，无用
/*
  drawings.setTime(ros::Time::now());
  model.lock();
  for(ObjectModel::iterator it = model.begin(); it != model.end(); ++it) {
    ObjectPtr object = *it;
  //  drawings.addMarkers(object->getVisualization(markers));// add by myself
    drawings.addMarkers(markers);
   drawings.setColor(1.0, 0.0, 0.0, drawings.markerArray.markers.back().color.a);
   drawings.drawCovariance(Eigen::Vector2f(object->getPosition().x(), object->getPosition().y()), object->getCovariance().block<2,2>(0,0));
  }
  model.unlock();
*/
//+++++++++++++++++==
  drawings.addMarkers(markers);
  drawings.sendAndResetData();
ROS_INFO("In objectTracker.cpp    after sendAndReserData");//12
}

} // namespace hector_object_tracker


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker");
ROS_ERROR("In  object_tracker.cpp    ~~first ");  // 1
  hector_object_tracker::ObjectTracker tracker;
ROS_ERROR("In  object_tracker.cpp    ~~Out object_tracker.cpp ");//8
  ros::spin();

  exit(0);
}
