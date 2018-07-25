# hector_worldmodel_msgs/ObjectInfo
# This message contains information about the estimated class affiliation, object id and corresponding support

# A string identifying the object's class (all objects of a class look the same)
string class_id

# A string identifying the specific object
string object_id

# A string that contains the name or a description of the specific object
string name

# The support (degree of belief) of the object's presence given as log odd ratio
float32 support

