
#ifndef OBJECT_TRACKER_OBJECT_H
#define OBJECT_TRACKER_OBJECT_H

#include <hector_object_tracker/types.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <map>

namespace hector_object_tracker {

class Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef boost::shared_ptr<Object> Ptr;
    typedef boost::shared_ptr<Object const> ConstPtr;
    typedef hector_worldmodel_msgs::ObjectState::_state_type StateType;

    Object(const std::string class_id = "", const std::string object_id = "");
    Object(const hector_worldmodel_msgs::Object& other);
    virtual ~Object();

    static void reset();

    void getMessage(hector_worldmodel_msgs::Object& object) const;
    hector_worldmodel_msgs::Object getMessage() const;
    void getVisualization(visualization_msgs::MarkerArray &markers) const;

    void getPoseWithCovariance(geometry_msgs::PoseWithCovariance& pose) const;
    void setPose(const geometry_msgs::PoseWithCovariance& pose);

    void getPose(geometry_msgs::Pose& pose) const;
    void getPose(tf::Pose& pose) const;
    void setPose(const geometry_msgs::Pose& pose);
    void setPose(const tf::Pose& pose);

    const Eigen::Vector3f& getPosition() const;
    void setPosition(const Eigen::Vector3f& position);
    void setPosition(const geometry_msgs::Point& position);
    void setPosition(const tf::Point& point);

    const Eigen::Quaternionf& getOrientation() const;
    void setOrientation(const geometry_msgs::Quaternion& orientation);
    void setOrientation(const tf::Quaternion& orientation);

    const Eigen::Matrix3f& getCovariance() const;
    void getCovariance(geometry_msgs::PoseWithCovariance::_covariance_type& covariance) const;
    void setCovariance(const Eigen::Matrix3f& covariance);
    void setCovariance(const tf::Matrix3x3& covariance);
    void setCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& covariance);

    const std::string& getClassId() const {
      return this->info.class_id;
    }

    const std::string& getObjectId() const {
      return this->info.object_id;
    }

    void setObjectId(const std::string& object_id) {
      this->info.object_id = object_id;
    }

    float getSupport() const {
      return this->info.support;
    }

    void setSupport(float support) {
      this->info.support = support;
    }

    void addSupport(float support) {
      this->info.support += support;
    }

    StateType getState() const {
      return this->state.state;
    }

    void setState(const StateType& state);

    const std::string& getName() const {
      return this->info.name;
    }

    void setName(const std::string& name) {
      this->info.name = name;
    }

    std_msgs::Header getHeader() const {
      return this->header;
    }

    void setHeader(const std_msgs::Header &header) {
      this->header = header;
    }

    ros::Time getStamp() const {
      return this->header.stamp;
    }

    void intersect(const tf::Pose& poseB, const Eigen::Matrix3f& covarianceB, float support);
    void update(const tf::Pose& poseB, const Eigen::Matrix3f& covarianceB, float support);
    void updateOrientation(const tf::Quaternion& orientationB, double slerp_factor);

    static void setNamespace(const std::string& ns);

    Object& operator=(const hector_worldmodel_msgs::Object& other);

    ObjectPtr transform(tf::Transformer& tf, const std::string& target_frame) const;
    ObjectPtr transform(tf::Transformer& tf, const std::string& target_frame, const ros::Time& target_time) const;

    double getDistance(const Object &other);

private:
    ros::NodeHandle nh;
    std_msgs::Header header;
    ObjectInfo info;
    ObjectState state;

    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    Eigen::Matrix3f covariance;

    static std::map<std::string,unsigned int> object_count;
    static std::string object_namespace;
};

} // namespace hector_object_tracker

#endif // OBJECT_TRACKER_OBJECT_H
