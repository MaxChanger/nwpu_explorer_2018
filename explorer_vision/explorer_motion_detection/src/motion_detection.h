#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

//#include <hector_worldmodel_msgs/ImagePercept.h>
#include <image_transport/image_transport.h>
#include <cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
//int count=0;

class MotionDetection
{
public:
    MotionDetection();
    ~MotionDetection();
    Mat MoveDetect(Mat frame1, Mat frame2);

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam_info);
    ros::Publisher image_percept_pub_;
    image_transport::CameraSubscriber camera_sub_;
    image_transport::CameraPublisher image_motion_pub_;
    image_transport::CameraPublisher image_detected_pub_;

    image_transport::CameraSubscriber image_sub_;

    //dynamic_reconfigure::Server<MotionDetectionConfig> dyn_rec_server_;

    cv_bridge::CvImageConstPtr img_prev_ptr_;
    cv_bridge::CvImageConstPtr img_current_ptr_;
    cv_bridge::CvImageConstPtr img_current_col_ptr_;

    int motion_detect_threshold_; //threshold image value
    double min_percept_size, max_percept_size;
    double min_density;
    std::string percept_class_id_;
    std::string sub_image_topic_;
    int count;
    Mat background;
};