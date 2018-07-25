# hector_worldmodel_msgs/PosePercept
# This message represents an observation of an object in a single image.

# The header should equal the header of the corresponding image.
Header header

# The estimated pose of the object with its covariance
geometry_msgs/PoseWithCovariance pose

# Additional information about the percept
PerceptInfo info
