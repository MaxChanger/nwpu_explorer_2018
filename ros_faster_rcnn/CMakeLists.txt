cmake_minimum_required(VERSION 2.8.3)
project(ros_faster_rcnn)

find_package(catkin REQUIRED
	roslaunch
	message_generation
	std_msgs
	cv_bridge
	sensor_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
	FILES
	Detection.msg
	DetectionArray.msg
	DetectionFull.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(PROGRAMS src/simpleDetect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/imageLoader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
# install(TARGETS ROS-faster-rcnn ROS-faster-rcnn_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

roslaunch_add_file_check(launch)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ROS-faster-rcnn.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
