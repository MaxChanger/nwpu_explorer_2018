#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.h>
#include <ros/console.h>
using namespace std;
//using namespace cv;

class ImageWarp{
public:
    ImageWarp(ros::NodeHandle temp);
    ~ImageWarp();
    void imageCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info);
private:
    image_transport::CameraPublisher qrcode_image_publisher_;//发布器
    image_transport::CameraSubscriber camera_accept;//接收机
    ros::NodeHandle nh_; 
    image_transport::ImageTransport image_transport_; 
    std::string camera_name_; 
    std::string img_name_;
};  