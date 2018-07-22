#ifndef EXPLORER_APP_NEW_H
#define EXPLORER_APP_NEW_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rqt_gui_cpp/plugin.h>
#include <explorer_app_new/widget.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <QPainter>
#include <ros/master.h>
#include <explorer_app_new/ui_widget.h>
#include <std_msgs/Bool.h>
#include <explorer_app_msgs/app_arm_info.h>
#include <explorer_app_msgs/app_co2.h>
#include <explorer_app_msgs/app_robot_info.h>
#include <map>
#include <QFileDialog>
#include <QSet>
#include <QList>
#include <explorer_msgs/explorer_low_level_data.h>
using namespace  std;

namespace  explorer_app_new
{
class Explorer_App_New : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
 public:
    explicit Explorer_App_New(QObject *parent = 0);
    ~Explorer_App_New();
    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
signals:

public slots:
    void markVictim(int id);
    void pubstart();
    void pubstop();
    void pubup();
    void publeft();
    void pubright();
    void pubdown();

protected:
private:
   void getInit();

   void qr_code_CallBack(const std_msgs::String::ConstPtr &ptr);
   void victim_CallBack(const std_msgs::String::ConstPtr &ptr);
   void co2_dataCallBack(const explorer_app_msgs::app_co2 &msg);
   void tem_dataCallBack(const explorer_msgs::explorer_low_level_data &msg);
   void robot_infoCallBack(const explorer_app_msgs::app_robot_info::ConstPtr &ptr);

   void light_CallBack(const std_msgs::BoolConstPtr &ptr);

private slots:
   void widget_FullScreen();
private:
   Widget *widget_;
   Ui_Widget ui_widget_;

   ros::Publisher victim_pub_;

   ros::Subscriber qr_code_sub_;
   ros::Subscriber victim_sub_;
   ros::Subscriber robot_info_sub_;
   ros::Subscriber light_info_sub_;

   ros::Publisher pub_control_xtion;


   string last_victim_;
   cv::Mat conversion_mat_;

   int co2_msg_id;
   ros::Subscriber co2_data_sub_;
   int tem_msg_id;
   ros::Subscriber tem_data_sub_;
};
}

#endif //EXPLORER_APP_NEW_H
