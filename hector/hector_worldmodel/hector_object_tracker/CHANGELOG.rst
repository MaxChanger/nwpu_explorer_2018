^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_object_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2014-09-06)
------------------
* generalized min_distance_between_objects check for confirmed objects
* removed object class dependent code from the implementation part
* added cmake_modules dependency for the Eigen cmake config
* do no longer model victims twice if they were already confirmed
* set octomap_distance_warnings to ROS_DEBUG
* fixed qrcode mapping
* orientation of objects is now considered and filtered (optional)
* fixed publishing of not populized objects
* Contributors: Alexander Stumpf, Dorothea Koert, Johannes Meyer, Stefan Kohlbrecher

0.3.1 (2014-03-30)
------------------
* changed distance for negative update
* renamed hector_object_tracker::param() method to hector_object_tracker::parameter()
  This will hopefully fix the build failures on Jenkins for lucid binaries.
  See http://jenkins.ros.org/job/ros-fuerte-hector-worldmodel_binarydeb_lucid_amd64/132/console
* added missing target dependency to hector_nav_msgs_generate_messages_cpp
* Contributors: Johannes Meyer

0.3.0 (2013-09-03)
------------------
* catkinized stack hector_worldmodel
