# hector_worldmodel_msgs/PerceptInfo
# This message contains information about the estimated class and object identity 

# A string identifying the object's class (all objects of a class look the same)
string class_id

# The class association support of the observation
# The support is the log odd likelihood ratio given by log(p(y/observation y belongs to object of class class_id) / p(y/observation y is a false positive))
float32 class_support

# A string identifying a specific object
string object_id

# The object association support of the observation
# The support is the log odd likelihood ratio given by log(p(observation belongs to object object_id) / p(observation is false positive or belongs to another object))
float32 object_support

# A string that contains the name or a description of the specific object
string name
