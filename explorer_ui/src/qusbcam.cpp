#include "qusbcam.h"

namespace qusbcam       //qdebug()调试用
{
    
QUsbcam::QUsbcam(int argc, char** argv):
    init_argc(argc),init_argv(argv)
    {}

QUsbcam::QUsbcam():
    init_argc(0), init_argv(0)
    {}

QUsbcam::~QUsbcam()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

bool QUsbcam::init(QString topic_name)    //要记得先调用QString的构造函数将图像话题名称传进去
{
    if(!topic_name.data())
    {
        qDebug()<<"no valid name";
        return false;
    }
    else
    {
        topic_name_ = topic_name;
    }

    ros::init(init_argc, init_argv, "qusbcam_node");
    ros::start();

    start();
    return true;
}

void QUsbcam::run()
{
    std::string camera_topic_name_ = topic_name_.toStdString();

    ros::Rate loop_rate(30);
    ros::NodeHandle nh;             //需要做个是否有这个话题的处理
    ros::Subscriber image_sub = nh.subscribe(camera_topic_name_, 1, &QUsbcam::QUsbcamCallback, this);

    while(ros::ok())                //这是当ros没关闭的时候相机节点就会继续运行
    {
        ros::spinOnce();
        loop_rate.sleep();
        Q_EMIT UsbcamCallback();    //用这个调用：QObject::connect(&qusbcam, SIGNAL(UsbcamCallback()), this, SLOT(updateimage()));
    }
}

void QUsbcam::QUsbcamCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_image;
        cv_image = cv_bridge::toCvShare(msg, "bgr8");
        img = QImage((const uchar*)((cv_image->image).data), (cv_image->image).cols, (cv_image->image).rows, QImage::Format_RGB888);
        swappedimg = img.rgbSwapped();
    }
    catch(cv::Exception& e)
    {
        const char* err_msg = e.what(); 
        ROS_ERROR("we got a cv problem:%s", err_msg);
    }

}

}
