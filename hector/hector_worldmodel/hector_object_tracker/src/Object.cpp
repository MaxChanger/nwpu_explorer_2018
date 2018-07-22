
#include "Object.h"
#include "parameters.h"
#include <boost/lexical_cast.hpp>

#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

namespace hector_object_tracker {

std::map<std::string,unsigned int> Object::object_count;
std::string Object::object_namespace;

Object::Object(const std::string class_id, const std::string object_id)
{
  if (!class_id.empty()) {
    this->info.class_id = class_id;
  } else 
  {
    this->info.class_id = "object";
  }

  if (!object_id.empty()) {
    this->info.object_id = object_id;
  } else {
    this->info.object_id = this->info.class_id + "_" + boost::lexical_cast<std::string>(object_count[this->info.class_id]++);
  }

  // if (this->info.object_id[0] != '/') this->info.object_id = object_namespace + "/" + this->info.object_id;
}

Object::Object(const hector_worldmodel_msgs::Object& other) {
  *this = other;
}

Object::~Object()
{
  ROS_WARN("who kill me?");
}

void Object::reset() {
  object_count.clear();
}

void Object::getMessage(hector_worldmodel_msgs::Object& object) const {
  object.header = this->header;
  this->getPoseWithCovariance(object.pose);
  object.info   = this->info;
  object.state  = this->state;
}

hector_worldmodel_msgs::Object Object::getMessage() const {
  hector_worldmodel_msgs::Object object;
  getMessage(object);
  return object;
}

void Object::getPoseWithCovariance(geometry_msgs::PoseWithCovariance& pose) const {
  getPose(pose.pose);
  getCovariance(pose.covariance);
}

void Object::setPose(const geometry_msgs::PoseWithCovariance& pose) {
  setPose(pose.pose);
  setCovariance(pose.covariance);
}

void Object::getPose(geometry_msgs::Pose& pose) const
{
  pose.position.x = this->position.x();
  pose.position.y = this->position.y();
  pose.position.z = this->position.z();
  pose.orientation.w = this->orientation.w();
  pose.orientation.x = this->orientation.x();
  pose.orientation.y = this->orientation.y();
  pose.orientation.z = this->orientation.z();
}

void Object::getPose(tf::Pose& pose) const
{
  pose.getOrigin().setValue(this->position.x(), this->position.y(), this->position.z());
  pose.getBasis().setRotation(tf::Quaternion(this->orientation.x(), this->orientation.y(), this->orientation.z(), this->orientation.w()));
}

void Object::setPose(const geometry_msgs::Pose& pose)
{
  setPosition(pose.position);
  setOrientation(pose.orientation);
}

void Object::setPose(const tf::Pose &pose)
{
  setPosition(pose.getOrigin());
  tf::Quaternion rot;
  pose.getBasis().getRotation(rot);
  setOrientation(rot);
}

const Eigen::Vector3f& Object::getPosition() const {
  return position;
}

void Object::setPosition(const Eigen::Vector3f& position) {
  this->position = position;
}

void Object::setPosition(const geometry_msgs::Point& position) {
  this->position << position.x, position.y, position.z;
}

void Object::setPosition(const tf::Point& position)
{
  this->position << position.x(), position.y(), position.z();
}

const Eigen::Quaternionf& Object::getOrientation() const
{
  return this->orientation;
}

void Object::setOrientation(const geometry_msgs::Quaternion& orientation)
{
  this->orientation.coeffs() << orientation.x, orientation.y, orientation.z, orientation.w;
}

void Object::setOrientation(const tf::Quaternion& orientation)
{
  this->orientation.coeffs() << orientation.x(), orientation.y(), orientation.z(), orientation.w();
}

const Eigen::Matrix3f& Object::getCovariance() const {
  return covariance;
}

void Object::getCovariance(geometry_msgs::PoseWithCovariance::_covariance_type& covariance) const {
  covariance[0]  = this->covariance(0,0);
  covariance[1]  = this->covariance(0,1);
  covariance[2]  = this->covariance(0,2);
  covariance[6]  = this->covariance(1,0);
  covariance[7]  = this->covariance(1,1);
  covariance[8]  = this->covariance(1,2);
  covariance[12] = this->covariance(2,0);
  covariance[13] = this->covariance(2,1);
  covariance[14] = this->covariance(2,2);
}

void Object::setCovariance(const Eigen::Matrix3f& eigen) {
  this->covariance = eigen;
}

void Object::setCovariance(const tf::Matrix3x3& tf) {
  this->covariance << tf[0][0], tf[0][1], tf[0][2],
                      tf[1][0], tf[1][1], tf[1][2],
                      tf[2][0], tf[2][1], tf[2][2];
}

void Object::setCovariance(const geometry_msgs::PoseWithCovariance::_covariance_type& covariance) {
  this->covariance << covariance[0],  covariance[1],  covariance[2],
                      covariance[6],  covariance[7],  covariance[8],
                      covariance[12], covariance[13], covariance[14];
}

void Object::setState(const StateType& state) {
  if (this->state.state == state) return;
  ROS_INFO("Setting object state for %s to %s", getObjectId().c_str(), hector_worldmodel_msgs::getObjectStateString(state));
  this->state.state = state;
}

void Object::intersect(const tf::Pose& poseB, const Eigen::Matrix3f& covarianceB, float support) {
  Eigen::Vector3f positionB(poseB.getOrigin().x(), poseB.getOrigin().y(), poseB.getOrigin().z());
  tf::Quaternion orientationB = poseB.getRotation();
    // old cov/covariance is A , new cov/covIn is B
  float omega = 0.5f;

  Eigen::Matrix3f A(covariance.inverse() * omega);
  Eigen::Matrix3f B(covarianceB.inverse() * (1.0f - omega));
  double infA = 1./covariance.trace();
  double infB = 1./covarianceB.trace();

  covariance = (A + B).inverse();
  position = covariance * (A * position + B * positionB);
  updateOrientation(orientationB, infB / (infA + infB));

  setPosition(position);
  setCovariance(covariance);
  addSupport(support);
}

void Object::update(const tf::Pose& poseB, const Eigen::Matrix3f& covarianceB, float support) {
  Eigen::Vector3f positionB(poseB.getOrigin().x(), poseB.getOrigin().y(), poseB.getOrigin().z());
  tf::Quaternion orientationB = poseB.getRotation();
    // old information is A , new information is B

  Eigen::Matrix3f A(covariance.inverse());
  Eigen::Matrix3f B(covarianceB.inverse());
  double infA = 1./covariance.trace();
  double infB = 1./covarianceB.trace();

  covariance = (A + B).inverse();
  position = covariance * (A * position + B * positionB);
  updateOrientation(orientationB, infB / (infA + infB));

  setPosition(position);
  setCovariance(covariance);
  addSupport(support);
}

void Object::updateOrientation(const tf::Quaternion& orientationB, double slerp_factor) {
  // update orientation of objects using low-pass filtering
  if (parameter(_with_orientation, getClassId())) {
      tf::Quaternion q(orientation.x(), orientation.y(), orientation.z(), orientation.w());
      q.slerp(orientationB, slerp_factor);
      setOrientation(q);
  }
  // or simply set new orientation
  else {
    setOrientation(orientationB);
  }
}

void Object::getVisualization(visualization_msgs::MarkerArray &markers) const {
  visualization_msgs::Marker marker;
  std::string postfix;

  // default color
  marker.color = parameter(_marker_color, this->info.class_id);

  switch(this->state.state) {
    case hector_worldmodel_msgs::ObjectState::CONFIRMED:
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
      postfix = " (CONFIRMED)";
      break;
    case hector_worldmodel_msgs::ObjectState::DISCARDED:
      marker.color.a = 0.5;
      postfix = " (DISCARDED)";
      break;
    case hector_worldmodel_msgs::ObjectState::INACTIVE:
      marker.color.a = 0.5;
      postfix = " (INACTIVE)";
      break;
    case hector_worldmodel_msgs::ObjectState::UNKNOWN:
      marker.color.a = 0.5;
      break;
    case hector_worldmodel_msgs::ObjectState::PENDING:
      marker.color.a = 0.5;
      postfix = " (PENDING)";
      break;
    default:
      break;
  }

  marker.header = this->header;
  marker.action = visualization_msgs::Marker::ADD;
  getPose(marker.pose);
  marker.ns = this->info.class_id;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  // markers.markers.push_back(marker);

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  markers.markers.push_back(marker);

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.text = (!this->info.name.empty() ? this->info.name : this->info.object_id) + postfix;
  marker.scale.x = 0.0;
  marker.scale.y = 0.0;
  marker.scale.z = 0.1;
  marker.pose.position.z += 1.5 * marker.scale.z;
  markers.markers.push_back(marker);
}

void Object::setNamespace(const std::string &ns) {
  object_namespace = ns;
}

Object& Object::operator =(const hector_worldmodel_msgs::Object& other) {
  header = other.header;
  info = other.info;
  state = other.state;
  setPose(other.pose);
  return *this;
}

ObjectPtr Object::transform(tf::Transformer& tf, const std::string& target_frame) const
{
  return transform(tf, target_frame, this->header.stamp);
}

ObjectPtr Object::transform(tf::Transformer& tf, const std::string& target_frame, const ros::Time& target_time) const
{
  tf::StampedTransform transform;
  tf.lookupTransform(target_frame, this->header.frame_id, target_time, transform);

  ObjectPtr result(new Object(*this));

  tf::Pose pose;
  this->getPose(pose);

  // transform pose
  pose = transform * pose;
  result->setPose(pose);

  // rotate covariance matrix
  tf::Matrix3x3 rotation(transform.getBasis());
  tf::Matrix3x3 cov(covariance(0,0), covariance(0,1), covariance(0,2),
                    covariance(1,0), covariance(1,1), covariance(1,2),
                    covariance(2,0), covariance(2,1), covariance(2,2));
  result->setCovariance(rotation * cov * rotation.transpose());

  // set new frame_id
  result->header.frame_id = target_frame;

  return result;
}

double Object::getDistance(const Object &other)
{
  return (this->getPosition() - other.getPosition()).norm();
}

} // namespace hector_object_tracker
