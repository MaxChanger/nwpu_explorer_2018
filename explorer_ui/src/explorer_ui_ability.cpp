#include "explorer_ui_ability.h"
#include "ui_explorer_ui_ability.h"

explorer_ui_ability::explorer_ui_ability(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::explorer_ui_ability)
{
    ui->setupUi(this);
    QFont ft;
    ft.setPointSize(24);
    ui->co2_label->setFont(ft);
    ui->co2_lineedit->setFont(ft);
//对自定义消息进行注册
    qRegisterMetaType<explorer_msgs::explorer_co2>("explorer_msgs::explorer_co2");

    int argc = 0; char **argv = NULL;
    ros::init(argc, argv, "client_plug");
    if (!ros::master::check())
    {
        ROS_INFO("No master started!");
        this->close();
    }
    ros::start(); 
    // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    t = new explorer_thread();
    QObject::connect(t,SIGNAL(signal_co2(explorer_msgs::explorer_co2)),this,SLOT(topic_co2(explorer_msgs::explorer_co2)));
    t->start();
}

explorer_ui_ability::~explorer_ui_ability()
{
    delete ui;
}
void explorer_ui_ability::topic_co2(explorer_msgs::explorer_co2 ptr){
    ui->co2_lineedit->setText(QString::number(ptr.co2_data));
}
