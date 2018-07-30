
#ifndef qrcode_detection_H
#define qrcode_detection_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

namespace zbar {
  class ImageScanner;
}

namespace ws_qrcode_detection {

class qrcode_detection_impl {
public:
  qrcode_detection_impl(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~qrcode_detection_impl();

protected:
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& image);
  void leftinfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info);
  void rightinfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info);



private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;

  //image_transport::CameraSubscriber camera_subscriber_camera1_;
  //image_transport::CameraSubscriber camera_subscriber_camera2_;
  //ros::V_Subscriber sub_camera1;
  //ros::V_Subscriber sub_camera2;
  ros::Subscriber sub_camera1;
  ros::Subscriber sub_camera2;
  ros::Subscriber info_camera1;
  ros::Subscriber info_camera2;



  //
  //myadd
  // image_transport::CameraSubscriber camera_subscriber_front_left_camera_;
  // image_transport::CameraSubscriber camera_subscriber_front_right_camera_;
  // image_transport::CameraSubscriber camera_subscriber_back_left_camera_;
  // image_transport::CameraSubscriber camera_subscriber_back_right_camera_;
  //

  image_transport::CameraPublisher rotated_image_publisher_;
  image_transport::CameraPublisher qrcode_image_publisher_;

  ros::Publisher percept_publisher_;
  ros::Publisher find_qrcode_;
  ros::Publisher qrcode_publisher;

  zbar::ImageScanner *scanner_;

  tf::TransformListener *listener_;
  std::string rotation_source_frame_id_;
  std::string rotation_target_frame_id_;
  int rotation_image_size_;
  std::string pub_qrcode_topic_;
  std::string pub_image_qrcode_topic_;
  
  std::string sub_image_qrcode_topic_camera1_;
  //myadd
  std::string sub_image_qrcode_topic_camera2_;
  //
  //myadd
  // std::string sub_image_qrcode_topic_front_left_camera_;
  // std::string sub_image_qrcode_topic_front_right_camera_;
  // std::string sub_image_qrcode_topic_back_left_camera_;
  // std::string sub_image_qrcode_topic_back_right_camera_;
  //

  std::string pub_find_qrcode_topic_;

  std::string camera_info1_;
  std::string camera_info2_;
  std::string camera_frame_;

};

} // namespace qrcode_detection

#endif // qrcode_detection_H
