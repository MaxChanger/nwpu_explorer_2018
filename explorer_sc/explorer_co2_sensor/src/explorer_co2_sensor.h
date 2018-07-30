#ifndef EXPLORER_CO2_SENSOR_H
#define EXPLORER_CO2_SENSOR_H

#include <ros/ros.h>
#include <explorer_msgs/explorer_message.h>
#include <explorer_msgs/light_control.h>
#include <explorer_msgs/explorer_co2.h>
#include <explorer_msgs/explorer_low_level_data.h>

class ExplorerCo2Sensor
{
 public:
    ExplorerCo2Sensor(ros::NodeHandle node) ;
    ~ExplorerCo2Sensor() ;
    void start() ;
protected:
private:

    void sendCo2Request() ;

    bool lightControlSrv(explorer_msgs::light_control::Request &req ,
                         explorer_msgs::light_control::Response &resp);

    void co2MsgSub(const explorer_msgs::explorer_low_level_dataConstPtr &ptr) ;

private:
    ros::NodeHandle nh_ ;
    ros::ServiceServer light_srv_ ;
    ros::Publisher co2_pub_ ;
    ros::Publisher co2_app_pub_  ;
    ros::Subscriber co2_sub_ ;
    int co2_id_ ;
    int light_on_ ;
};

#endif // EXPLORER_CO2_SENSOR_H
