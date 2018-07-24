#include "explorer_co2_sensor.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <sstream>

using namespace std ;
using namespace cv;

ExplorerCo2Sensor::ExplorerCo2Sensor(ros::NodeHandle node)
    :nh_(node),
      light_on_(0),
      co2_id_(9)
{
    stringstream sub_name_ ;
    sub_name_<<"/explorer_serial_data/"<<co2_id_ ;
    light_srv_ = nh_.advertiseService("explorer_light_control",&ExplorerCo2Sensor::lightControlSrv,this) ;
    co2_pub_ = nh_.advertise<explorer_msgs::explorer_message>("/explorer_driver",10) ;
    co2_app_pub_ = nh_.advertise<explorer_msgs::explorer_co2>("/explorer_app_co2_data",10) ;
    co2_sub_ = nh_.subscribe(sub_name_.str() ,10, &ExplorerCo2Sensor::co2MsgSub,this) ;
}

void ExplorerCo2Sensor::co2MsgSub(const explorer_msgs::explorer_low_level_dataConstPtr &ptr)
{
    float data_1 = ptr->can_serial_data_1;
    //int data_2 = ptr->can_serial_data_2;
    explorer_msgs::explorer_co2 app_co2_message ;
    app_co2_message.head.stamp=ros::Time::now() ;
    app_co2_message.co2_data = data_1;
    co2_app_pub_.publish(app_co2_message);

    Mat temp(300,700,CV_8UC1,Scalar(255));
    stringstream ss;
    ss<<data_1; 
    string info = ss.str();
    //string info=toString(data_1);
    info="Co2:"+info + "ppm";
    putText(temp,info,Point(10,200),FONT_HERSHEY_SIMPLEX,3,Scalar(0),3);
    imshow("temp",temp);
    waitKey(4);

    ROS_INFO_STREAM("co2 data  = "<<app_co2_message.co2_data) ;
}

void ExplorerCo2Sensor::sendCo2Request()
{
    explorer_msgs::explorer_message drive_message ;
    drive_message.high_level_id = co2_id_ ;
    drive_message.low_level_id = co2_id_ ;
    drive_message.data.push_back(light_on_);
    drive_message.data.push_back(1);
    co2_pub_.publish(drive_message) ;
}

ExplorerCo2Sensor::~ExplorerCo2Sensor()
{
    nh_.shutdown();
}

bool ExplorerCo2Sensor::lightControlSrv(explorer_msgs::light_control::Request &req, explorer_msgs::light_control::Response &resp)
{
    ROS_INFO("control light") ; 
    if(req.light_on)
    {
        ROS_INFO("light on") ;
        light_on_ = 1 ;
    }
    else
    {
        ROS_INFO("light close") ;
        light_on_ = 0 ;
    }
    return true ;
}

void ExplorerCo2Sensor::start()
{
    ros::Rate r(10) ;
    while (ros::ok()) {
        sendCo2Request();
        ros::spinOnce() ; 
        r.sleep() ;
    }
}

int main(int argc ,char **argv)
{
    ros::init(argc , argv,"explorer_co2_sensor") ;
    ros::NodeHandle node ;
    ExplorerCo2Sensor sensor_control(node) ;
    sensor_control.start();
    return 0 ;
}
