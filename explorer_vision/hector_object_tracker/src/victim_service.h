#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
//#include <vector>
#include <tf/transform_listener.h>

#include <tf/tf.h>
#include <list>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <hector_object_tracker/types.h>
#include <hector_worldmodel_msgs/SetObjectState.h>
#include <hector_worldmodel_msgs/SetObjectName.h>
#include <hector_worldmodel_msgs/AddObject.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>


#include "ObjectModel.h"


using namespace std;

class VictimService{
public:
    VictimService();
    ~VictimService();
private:
    void VictimInfoCallback(const geometry_msgs::PoseStamped& pose);
    bool VictimServiceCallback(hector_worldmodel_msgs::GetObjectModel::Request& request, hector_worldmodel_msgs::GetObjectModel::Response& response);
    //std::vector<MergedModelPtr> victim_merged_models;

    ros::Subscriber victimSub;
    ros::ServiceServer victimSer;

    hector_object_tracker::ObjectModel *camera_right;
    hector_object_tracker::ObjectModel *camera_left;
    hector_object_tracker::ObjectModel merged;
    //list<hector_object_tracker::ObjectModel> oldObjs; 

    tf::TransformListener tf;
    Eigen::Vector2f last_left;
    Eigen::Vector2f last_right;

};