# hector_worldmodel_msgs/ImagePercept
# This message represents an observation of an object in a single image.

# The header should equal the header of the corresponding image.
Header header

# The camera info which is needed to project from image coordinates to world coordinates
sensor_msgs/CameraInfo camera_info

# Center coordinates of the percept in image coordinates
# x: axis points to the right in the image
# y: axis points downward in the image
float32 x
float32 y

# Normalized size of the percept in image coordinates (or 0.0)
float32 width
float32 height

# Distance estimate (slope distance) (or 0.0)
float32 distance

# Additional information about the percept
PerceptInfo info
