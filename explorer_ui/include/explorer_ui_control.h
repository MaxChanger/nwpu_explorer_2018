/*code is far away from bug with the animal protecting 
 *  ┏┓　　　┏┓ 
 *┏┛┻━━━┛┻┓ 
 *┃　　　　　　　┃ 　 
 *┃　　　━　　　┃ 
 *┃　┳┛　┗┳　┃ 
 *┃　　　　　　　┃ 
 *┃　　　┻　　　┃ 
 *┃　　　　　　　┃ 
 *┗━┓　　　┏━┛ 
 *　　┃　　　┃神兽保佑 
 *　　┃　　　┃代码无BUG！ 
 *　　┃　　　┗━━━┓ 
 *　　┃　　　　　　　┣┓ 
 *　　┃　　　　　　　┏┛ 
 *　　┗┓┓┏━┳┓┏┛ 
 *　　　┃┫┫　┃┫┫ 
 *　　　┗┻┛　┗┻┛  
 *　　　 
 */  

#ifndef EXPLORER_UI_CONTROL_H
#define EXPLORER_UI_CONTROL_H

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <explorer_msgs/explorer_vice_wheel.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#endif

#include "explorer_thread.h"

#include <QMainWindow>
#include <QString>

namespace Ui {
class explorer_ui_control;
}

class explorer_ui_control : public QMainWindow
{
    Q_OBJECT

public:
    explicit explorer_ui_control(QWidget *parent = 0);
    ~explorer_ui_control();

private:
    Ui::explorer_ui_control *ui;
    //两个通过slider来进行速度更改的发布器
    //界面控制端接口
    //两个发布器 两个
    ros::Publisher ui_wheel_slider_pub;
    ros::Publisher ui_vice_wheel_slider_pub;

    geometry_msgs::Twist vel_publisher,last_vel_published;
    explorer_msgs::explorer_vice_wheel vice_vel_publisher,last_vice_vel_publisher;

    ros::Subscriber topic_vice_wheel_sub;
    ros::Subscriber topic_wheel_sub;
    explorer_thread *t;
    double wheel_separation;
    double left_up_vice_init,right_up_vice_init,left_down_vice_init,right_down_vice_init;
private Q_SLOTS :
    //wheel
    void ui_wheel_left_speed_change(int value);
    void ui_wheel_up_speed_change(int value);
    //vice_wheel
    void ui_up_right_vice_wheel_change(int value);
    void ui_up_left_vice_wheel_change(int value);
    void ui_down_right_vice_wheel_change(int value);
    void ui_down_left_vice_wheel_change(int value);

    //states:
    //从另一个线程里面接受到的消息
    void topic_vice_wheel_change(explorer_msgs::explorer_vice_wheel);
    void topic_wheel_change(geometry_msgs::Twist);
    void topic_arm_change(sensor_msgs::JointState);

    void on_check_clicked(); 
    //connect
   /* void connect_black();
    void connect_white();
    void ui_control_start();
    void ui_control_stop();*/
};

#endif // EXPLORER_UI_CONTROL_H
