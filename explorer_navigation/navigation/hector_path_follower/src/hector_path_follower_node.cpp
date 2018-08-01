#include "hector_path_follower/hector_path_follower.h"

#include <ros/ros.h>
#include <tf/tf.h>



int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  tf::TransformListener* tfl = new tf::TransformListener();

  pose_follower::HectorPathFollower pf;
  pf.initialize(tfl);

  ros::spin();

  delete tfl;

  return 0;
}
