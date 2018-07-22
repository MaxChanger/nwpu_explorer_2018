# hector_worldmodel_msgs/Object
# This message represents an estimate of an object's pose and identity.

# The header.
#   stamp: Timestamp of last update.
#   frame_id: Coordinate frame, in which the pose is given
Header header

# The pose
geometry_msgs/PoseWithCovariance pose

# Further information about the object
ObjectInfo info

# The tracked state of the object
ObjectState state
