
#include <hector_worldmodel_msgs/Object.h>
#include <hector_worldmodel_msgs/ObjectModel.h>
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <hector_worldmodel_msgs/PosePercept.h>
#include <hector_worldmodel_msgs/constants/ObjectState.h>

#include <boost/shared_ptr.hpp>
#include <list>

#ifndef HECTOR_OBJECT_TRACKER_TYPES_H
#define HECTOR_OBJECT_TRACKER_TYPES_H

namespace hector_object_tracker {

  class Object;
  typedef boost::shared_ptr<Object> ObjectPtr;
  typedef boost::shared_ptr<Object const> ObjectConstPtr;
  typedef std::list<ObjectPtr> ObjectList;

  class ObjectModel;

  using hector_worldmodel_msgs::ObjectState;
  using hector_worldmodel_msgs::ObjectInfo;
  using hector_worldmodel_msgs::ImagePercept;
  using hector_worldmodel_msgs::PosePercept;

}

#endif // HECTOR_WORLDMODEL_TYPES_H
