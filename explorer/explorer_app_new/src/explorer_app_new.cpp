#include "explorer_app_new/explorer_app_new.h"
#include "explorer_app_new/light_warn.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/core/core.hpp>
#include <QMessageBox>
#include <strstream>
#include <sstream>
namespace explorer_app_new {
Explorer_App_New::Explorer_App_New(QObject *parent):
    rqt_gui_cpp::Plugin(),
    widget_(0) {
    setObjectName("Team Explorer App");
}

Explorer_App_New::~Explorer_App_New() {

}

void Explorer_App_New::initPlugin(qt_gui_cpp::PluginContext &context) {
    widget_ = new Widget();
    ui_widget_.setupUi(widget_);
    getInit();
    context.addWidget(widget_);

    tem_msg_id = 8;
    co2_data_sub_ = getNodeHandle().subscribe("/explorer_app_co2_data", 10, &Explorer_App_New::co2_dataCallBack, this);
    stringstream sub_tem_name_;
    sub_tem_name_ << "/explorer_serial_data/" << tem_msg_id;
    tem_data_sub_ = getNodeHandle().subscribe(sub_tem_name_.str(), 10, &Explorer_App_New::tem_dataCallBack, this);
    //robot_info_sub_ = getNodeHandle().subscribe("/explorer_robot_info",10,&Explorer_App_New::robot_infoCallBack,this);

    victim_sub_ = getNodeHandle().subscribe("/find_victim", 10, &Explorer_App_New::victim_CallBack, this);
    qr_code_sub_ = getNodeHandle().subscribe("/find_qrcode", 10, &Explorer_App_New::qr_code_CallBack, this);
    light_info_sub_ = getNodeHandle().subscribe("/explorer_light_info", 10, &Explorer_App_New::light_CallBack, this);

    victim_pub_ = getNodeHandle().advertise<std_msgs::String>("/add_victim", 10);

    pub_control_xtion = getNodeHandle().advertise<std_msgs::String>("/pub_control", 100);
}

void Explorer_App_New::light_CallBack(const std_msgs::BoolConstPtr &ptr) {
    //widget_->changeRobotLight(ui_widget_.label_3,ptr->data);
}

void Explorer_App_New::markVictim(int id) {
    std_msgs::String current_data_;

    if (id == 3) {
        current_data_.data = "add_no_victim";
    } else {
        current_data_.data = "add_victim";
    }

    ROS_INFO("publish data");
    victim_pub_.publish(current_data_);
}

void Explorer_App_New::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const {

}

void Explorer_App_New::shutdownPlugin() {

}

void Explorer_App_New::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings) {

}

void Explorer_App_New::qr_code_CallBack(const std_msgs::String::ConstPtr &ptr) {
    ROS_INFO("qr_code_find");

    if (!widget_->qr_button_warn__->widget_flicker_) {
        ui_widget_.qr_code_find_button->setText("find the qr code");
        widget_->qr_button_warn__->widgetFlicker();
        widget_->qr_label_warn__->widgetFlicker();
    }
}

void Explorer_App_New::victim_CallBack(const std_msgs::String::ConstPtr &ptr) {
    ROS_INFO("victim callback");
    static int i = 0;
    string current_victim = ptr->data;

    if ((i == 0) || (last_victim_ != current_victim)) {
        if (!widget_->victim_button_warn__->widget_flicker_) {
            ui_widget_.victim_find_button->setText("find the victim");
            widget_->victim_button_warn__->widgetFlicker();
            widget_->victim_label_warn__->widgetFlicker();
        }
    }

    i++;
    last_victim_ = current_victim;
}
void Explorer_App_New::co2_dataCallBack(const explorer_app_msgs::app_co2 &msg) {
    QString current_co2_data;
    current_co2_data.setNum(msg.co2_data);
    ui_widget_.co2_num_show->display(current_co2_data);
}
void Explorer_App_New::tem_dataCallBack(const explorer_msgs::explorer_low_level_data &msg) {
    QString current_aim_tem_data;
    current_aim_tem_data.setNum(((int)(msg.can_serial_data_1*100))/100.0);
    ui_widget_.aim_t_num_show->display(current_aim_tem_data);
    QString current_env_tem_data;
    current_env_tem_data.setNum(((int)(msg.can_serial_data_2*100*10e34/2))/100.0);
    ui_widget_.env_t_num_show->display(current_env_tem_data);
}


void Explorer_App_New::getInit() {
    widget_->widget_width_ = widget_->width();
    widget_->widget_height_ = widget_->height();


    widget_->victim_choose_widget = new VictimChoose() ;

    widget_->qr_label_warn__ = new LabelWarn(ui_widget_.qr_code_find_lable, ":/new/prefix1/no_info.png", ":/new/prefix1/get_info.png");
    widget_->qr_button_warn__ = new ButtonWarn(ui_widget_.qr_code_find_button, "font: 75 18pt \"Padauk Book\";");
    widget_->victim_label_warn__ = new LabelWarn(ui_widget_.victim_find_lable, ":/new/prefix1/no_info.png", ":/new/prefix1/get_info.png");
    widget_->victim_button_warn__ = new ButtonWarn(ui_widget_.victim_find_button, "font: 75 18pt \"Padauk Book\";");


    connect(ui_widget_.qr_code_find_button, SIGNAL(clicked()), widget_, SLOT(qr_codeMessageBox()));
    connect(ui_widget_.victim_find_button, SIGNAL(clicked()), widget_, SLOT(victim_chooseOpen()));
    connect(widget_->victim_choose_widget, SIGNAL(sendVictim(int)), this, SLOT(markVictim(int)));

}

void Explorer_App_New::widget_FullScreen() {
    widget_->showFullScreen();
}
PLUGINLIB_EXPORT_CLASS(explorer_app_new::Explorer_App_New, rqt_gui_cpp::Plugin)
}

