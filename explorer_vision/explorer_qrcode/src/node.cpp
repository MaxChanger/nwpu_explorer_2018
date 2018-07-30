#include <explorer_qrcode/qrcode_detection.h>
using namespace ws_qrcode_detection;
int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  qrcode_detection_impl detector(ros::NodeHandle(), ros::NodeHandle("~"));
  ROS_ERROR("In node.cpp   main()");
  ros::spin();
  return 0;
}
