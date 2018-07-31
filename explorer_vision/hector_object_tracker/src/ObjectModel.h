
#ifndef OBJECT_TRACKER_OBJECTMODEL_H
#define OBJECT_TRACKER_OBJECTMODEL_H

#include <hector_object_tracker/types.h>

#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <string>
#include <boost/thread/recursive_mutex.hpp>

#include <Eigen/Geometry>

#include <tf/transform_listener.h>

namespace hector_object_tracker {

class ObjectModel
{
public:
    typedef ObjectList::iterator iterator;
    typedef ObjectList::const_iterator const_iterator;

public:
    ObjectModel(const std::string& frame_id = std::string());
    ObjectModel(const ObjectModel&);
    virtual ~ObjectModel();

    ObjectList getObjects() const;
    ObjectList getObjects(const std::string& class_id) const;
    ObjectPtr getObject(const std::string& object_id) const;

    std_msgs::Header getHeader() const;
    void setFrameId(const std::string &frame_id);

    void getMessage(hector_worldmodel_msgs::ObjectModel& model) const;
    hector_worldmodel_msgs::ObjectModelPtr getMessage() const;
    void getVisualization(visualization_msgs::MarkerArray &markers) const;
    void reset();

    iterator begin() { return objects.begin(); }
    iterator end()   { return objects.end(); }
    const_iterator begin() const { return objects.begin(); }
    const_iterator end() const { return objects.end(); }

    ObjectPtr create(const std::string& class_id = "", const std::string& object_id = "");
    ObjectPtr add(const std::string& class_id = "", const std::string& object_id = "");
    ObjectPtr add(ObjectPtr object);
    void remove(ObjectPtr object);
    void remove(iterator it);

    ObjectModel &operator=(const ObjectModel& other);
    ObjectModel &operator=(const hector_worldmodel_msgs::ObjectModel& other);

    void lock() const { objectsMutex.lock(); }
    bool try_lock() const { return objectsMutex.try_lock(); }
    void unlock() const { objectsMutex.unlock(); }

    float getBestCorrespondence(ObjectPtr &object, const tf::Pose& pose, const Eigen::Matrix3f& covariance, const std::string& class_id, const std::string& name, float max_distance = 0.0) const;

    void mergeWith(const ObjectModel& other, tf::TransformListener& tf, const std::string& prefix = std::string());
    void merge(const ObjectPtr& other, tf::TransformListener& tf, const std::string& prefix = std::string());

    int modelsize();
    //void cloneObjects();
    void initMerged();
private:
    std_msgs::Header header;
    ObjectList objects;
    mutable boost::recursive_mutex objectsMutex;
};

} // namespace hector_object_tracker

#endif // OBJECT_TRACKER_OBJECTMODEL_H
