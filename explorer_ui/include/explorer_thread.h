#ifndef EXPLORER_THREAD_H
#define EXPLORER_THREAD_H

#ifndef Q_MOC_RUN

#include <iostream>
#include <ros/ros.h>
#include <explorer_msgs/explorer_vice_wheel.h>
#include <geometry_msgs/Twist.h>
#include <explorer_msgs/explorer_co2.h>
#include <sensor_msgs/JointState.h>

#endif

#include <QThread>
#include <QString>

class explorer_thread :public QThread{


    Q_OBJECT
public:
    explicit explorer_thread(QObject *parent = 0);
    //Thread();
    //进行接受消息内容的设置
    void setThetopic();
    void stop();
    void topic_wheel_change(geometry_msgs::Twist);
    void topic_vice_wheel_change(explorer_msgs::explorer_vice_wheel);
    void topic_arm_change(sensor_msgs::JointState);
    void topic_co2_change(explorer_msgs::explorer_co2);
protected:
//在run里面实现消息的监听机制
    void run();


private:
    //QString message;
    
    volatile bool stopped;
Q_SIGNALS:
    void signal_wheel(geometry_msgs::Twist);
    void signal_vice_wheel(explorer_msgs::explorer_vice_wheel);
    void signal_arm(sensor_msgs::JointState);
    void signal_co2 (explorer_msgs::explorer_co2);
};


#endif 
