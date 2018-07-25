#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_warp.h"  
#include <ros/console.h>


using namespace cv;
using namespace std;


ImageWarp::ImageWarp(ros::NodeHandle temp):nh_(),image_transport_(temp)
{
    ros::NodeHandle local("~");
    local.param("camera_name",camera_name_,std::string("/camera/image"));
    local.param("img_name",img_name_,std::string("/camera_left/image_raw"));
    qrcode_image_publisher_ = image_transport_.advertiseCamera(camera_name_, 10);
    camera_accept = image_transport_.subscribeCamera(img_name_, 10, &ImageWarp::imageCallback, this);
    ROS_INFO("init the node");
}

ImageWarp::~ImageWarp(){}

void ImageWarp::imageCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    try
    {
        Mat matwrap;
        //Mat src
        cv_bridge::CvImageConstPtr cv_image;
        cv_image=cv_bridge::toCvShare(msg, "bgr8");
        Mat src=cv_image->image.clone();
        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        Mat warp_dst = Mat::zeros( src.rows, src.cols, src.type() );
        Mat warp_mat;

        Point2f srcTri[3];
        Point2f dstTri[3];

   /// 设置源图像和目标图像上的三组点以计算仿射变换
        srcTri[0] = Point2f( 0,0 );
        srcTri[1] = Point2f( src.cols - 1, 0 );
        srcTri[2] = Point2f( 0, src.rows - 1 );


        dstTri[0] = Point2f( src.cols-1, src.rows-1 );
        dstTri[1] = Point2f( 0, src.rows-1 );
        dstTri[2] = Point2f( src.cols-1, 0 );

   /// 求得仿射变换
        warp_mat = getAffineTransform( srcTri, dstTri );

   /// 对源图像应用上面求得的仿射变换
        warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
        warp_dst.copyTo(matwrap);
//    cv::imshow("view1", warp_dst);
//         cv::waitKey(30);


    //我只是一个平凡的视频输出器，这个主要用于调试，不用管
    if (qrcode_image_publisher_.getNumSubscribers() > 0)//&&msg->header.frame_id=="camera_right")
    {
      try
      {
        //cv::Rect rect(cv::Point2i(std::max(min_x, 0), std::max(min_y, 0)), cv::Point2i(std::min(max_x, cv_image->image.cols), std::min(max_y, cv_image->image.rows)));
        cv_bridge::CvImagePtr qrcode_cv(new cv_bridge::CvImage(*cv_image));
        //qrcode_cv->image = cv_image->image(rect);
        qrcode_cv->image=matwrap;

        sensor_msgs::Image qrcode_image;
        qrcode_cv->toImageMsg(qrcode_image);
        qrcode_image_publisher_.publish(qrcode_image, *camera_info);
      }
      catch (cv::Exception &e)
      {
        ROS_ERROR("cv::Exception: %s", e.what());
      }
    }

    }
    catch (cv_bridge::Exception& e)  
    {  
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
    } 


}




// int main(int argc, char **argv)
// {

//     ros::init(argc, argv, "image_warp");

//     ros::NodeHandle nh;

//     // cv::namedWindow("view");
//     // cv::namedWindow("view1");
//     // cv::startWindowThread();

//     image_transport::ImageTransport it(nh);
//     image_transport::ImageTransport it1(nh);
//     image_transport::Publisher pub = it1.advertise("camera_arm/image_raw_warpped", 1);
//     cv_bridge::CvImagePtr frame;
//     frame = boost::make_shared<cv_bridge::CvImage>();
//     frame->encoding = sensor_msgs::image_encodings::BGR8;


//     while(ros::ok())
//     {
//         image_transport::Subscriber sub = it.subscribe("camera_arm/image_raw", 1, imageCallback);
//         frame->image=matwrap;
//         if(matwrap.empty()) cout<<"!!!!"<<endl;
//         waitKey(30);
//         if(!(frame->image.empty())) pub.publish(frame->toImageMsg());

//         ros::spinOnce();
//     }


//     return 0;
// }
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_wrap");
  ImageWarp tran( ros::NodeHandle("") );
  ROS_ERROR("In node.cpp   main()");
  ros::spin();
  return 0;
}