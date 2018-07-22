#include <cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "motion_detection.h"
#include<ros/ros.h>
using namespace std;
using namespace cv;

//int count=0;

MotionDetection::MotionDetection()
{
    ROS_ERROR("ssss");
    ros::NodeHandle n;
    ros::NodeHandle p_n("~");
    img_prev_ptr_.reset();
    img_current_ptr_.reset();

    image_transport::ImageTransport it(n);
    image_transport::ImageTransport image_motion_it(p_n);
    image_transport::ImageTransport image_detected_it(p_n);

    p_n.param("sub_image_topic", sub_image_topic_, std::string("/usb_cam/image_raw"));

    image_sub_ = it.subscribeCamera(sub_image_topic_, 10, &MotionDetection::imageCallback, this);

    //    image_percept_pub_ = n.advertise<motion_detection::ImagePercept>("image_percept", 20);
    image_motion_pub_ = image_motion_it.advertiseCamera("image_motion", 10);
    image_detected_pub_ = image_detected_it.advertiseCamera("image_detected", 10);
    count=0;
    ROS_ERROR("start !!!");
}

MotionDetection::~MotionDetection(){};

Mat MotionDetection::MoveDetect(Mat frame1, Mat frame2)
{
    Mat result = frame2.clone();
    Mat gray1, gray2;
    ROS_INFO("channel are %d ,%d",frame1.channels(),frame2.channels());
    cvtColor(frame1, gray1, CV_BGR2GRAY);
    cvtColor(frame2, gray2, CV_BGR2GRAY);

    Mat diff;
    absdiff(gray1, gray2, diff);
    //imshow("absdiss", diff);
    threshold(diff, diff, 30, 255, CV_THRESH_BINARY);
    //imshow("threshold", diff);
 
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat element2 = getStructuringElement(MORPH_RECT, Size(25, 25));
    erode(diff, diff, element);
    //imshow("erode", diff);
 
    dilate(diff, diff, element2);
    //imshow("dilate", diff);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarcy;
    //画椭圆及中心
    findContours(diff, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cout<<"num="<<contours.size()<<endl;
    vector<RotatedRect> box(contours.size());
    for(int i=0; i<contours.size(); i++)
    {
        box[i] = fitEllipse(Mat(contours[i]));
        ellipse(result, box[i], Scalar(0, 255, 0), 2, 8);    
        circle(result, box[i].center, 3, Scalar(0, 0, 255), -1, 8);
    }
    ROS_INFO("end the function!");
    return result;
}


void MotionDetection::imageCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &cam_info)
{
     cv_bridge::CvImageConstPtr img_next_ptr(cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8));

    if (img_current_col_ptr_)
    {
        Mat frame;
        Mat result;
        //static int count=0;
        //cap>>frame;
        frame=img_current_col_ptr_->image.clone();
        if(frame.empty())
            return;
        else
        {            
            count++;
            cout<<"have call callback function:"<<count<<endl;
            if(count%5==1)
                 background = frame.clone(); //提取第一帧为背景帧
            if(count>10000)
                count=1;
            //imshow("video", background);
            ROS_INFO("start the MoveDetect with %d channel",frame.channels());
            result = MotionDetection::MoveDetect(background, frame);
            ROS_ERROR("out the MoveDetect");
            imshow("result", result);
            waitKey(50);
        }
        if (image_detected_pub_.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage cvImg;
            result.copyTo(cvImg.image);
            cvImg.header = img->header;
            cvImg.encoding = sensor_msgs::image_encodings::BGR8;
            image_detected_pub_.publish(cvImg.toImageMsg(), cam_info); // info -> cam_info
        }
    }
    img_prev_ptr_ = img_current_ptr_;
    img_current_ptr_ = img_next_ptr;

    img_current_col_ptr_ = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"motion detect");
    ROS_ERROR("Run the programe");
    MotionDetection md;
    ros::spin();
    return 0;
}
