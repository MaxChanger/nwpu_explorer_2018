#include "explorer_ui_control.h"
#include "ui_explorer_ui_control.h"


//需要多个窗口动态调整,四个摄像头的数据到时候可以直接拉动调整大小
explorer_ui_control::explorer_ui_control(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::explorer_ui_control)
{
    
    
    //oh,Qt begin working!!
    ui->setupUi(this);
    wheel_separation = 1.2;
    //进行通讯数据的声明
    qRegisterMetaType<explorer_msgs::explorer_vice_wheel>("explorer_msgs::explorer_vice_wheel");
    qRegisterMetaType<geometry_msgs::Twist>("geometry_msgs::Twist");
    qRegisterMetaType<sensor_msgs::JointState>("sensor_msgs::JointState");
    
    //对滑块的初始位置和具体的范围进行限定
    //副履带的初始值和限位可以进行参数服务器读取

    ui->up_down_wheel_slider->setValue(0);
    ui->up_down_wheel_slider->setMinimum(-100);
    ui->up_down_wheel_slider->setMaximum(100);
    
    ui->left_right_wheel_slider->setValue(0);
    ui->left_right_wheel_slider->setMinimum(-100);
    ui->left_right_wheel_slider->setMaximum(100);

    ui->left_right_wheel_cmd_lineedit->setText(QString::number(ui->left_right_wheel_slider->value()));
    ui->up_down_wheel_cmd_lineedit->setText(QString::number(ui->up_down_wheel_slider->value()));

    //wheel speed change
    QObject::connect(ui->left_right_wheel_slider,SIGNAL(valueChanged(int)),this,SLOT(ui_wheel_left_speed_change(int)));
    QObject::connect(ui->up_down_wheel_slider,SIGNAL(valueChanged(int)),this,SLOT(ui_wheel_up_speed_change(int)));

    //vice wheel speed change
    QObject::connect(ui->up_right_vice_wheel_cmd_slider,SIGNAL(valueChanged(int)),this,SLOT(ui_up_right_vice_wheel_change(int)));
    QObject::connect(ui->up_left_vice_wheel_cmd_slider,SIGNAL(valueChanged(int)),this,SLOT(ui_up_left_vice_wheel_change(int)));
    QObject::connect(ui->down_right_vice_wheel_cmd_slider,SIGNAL(valueChanged(int)),this,SLOT(ui_down_right_vice_wheel_change(int)));
    QObject::connect(ui->down_left_vice_wheel_cmd_slider,SIGNAL(valueChanged(int)),this,SLOT(ui_down_left_vice_wheel_change(int)));
    //进行副履带的数据初始化
    left_up_vice_init = right_up_vice_init =0 ;
    left_down_vice_init = left_down_vice_init =0 ;
    //oh,ros bagin!!!!!!
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

    ui_wheel_slider_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ui_vice_wheel_slider_pub = n.advertise<explorer_msgs::explorer_vice_wheel>("explorer_vice_wheel",1);
    //使用另一个线程进行ros数据的收集
    t = new explorer_thread();
    QObject::connect(t,SIGNAL(signal_wheel(geometry_msgs::Twist)),this,SLOT(topic_wheel_change(geometry_msgs::Twist)));
    QObject::connect(t,SIGNAL(signal_vice_wheel(explorer_msgs::explorer_vice_wheel)),this,SLOT(topic_vice_wheel_change(explorer_msgs::explorer_vice_wheel)));
    QObject::connect(t,SIGNAL(signal_arm(sensor_msgs::JointState)),this,SLOT(topic_arm_change(sensor_msgs::JointState)));
    t->start();
}

explorer_ui_control::~explorer_ui_control()
{
    delete ui;
}


//后端逻辑  建议与UI分离
//*************get message and show the message to the srceen (by topic )*****************
//show the wheel speed
void explorer_ui_control::topic_wheel_change(geometry_msgs::Twist ptr){
    ui->left_wheel_lineedit->setText(QString::number(ptr.linear.x-ptr.linear.y-ptr.angular.z*wheel_separation/2.0)) ;
    ui->right_wheel_lineedit->setText(QString::number(ptr.linear.x + ptr.linear.y + ptr.angular.z * wheel_separation /2.0)) ;
    //get the message by the topic and show the message  
}

void explorer_ui_control::topic_vice_wheel_change(explorer_msgs::explorer_vice_wheel ptr){
    left_up_vice_init +=ptr.front_left_wheel_angular;
    right_up_vice_init +=ptr.back_right_wheel_angular;
    right_down_vice_init +=ptr.front_right_wheel_angular;
    left_down_vice_init +=ptr.back_left_wheel_angular;
    ui->front_right_vice_wheel_lineedit->setText(QString::number(right_up_vice_init));
    ui->back_right_vice_wheel_lineedit->setText(QString::number(right_down_vice_init));
    ui->front_left_vice_wheel_lineedit->setText(QString::number(left_up_vice_init));
    ui->back_left_vice_wheel_lineedit->setText(QString::number(left_down_vice_init));
}
//the speed is too high
//可能需要调整这几个的次序
void explorer_ui_control::topic_arm_change(sensor_msgs::JointState ptr){
    //0整体左右 1第一个上下 2第二个上下 3第一个旋转 4第二个摆动 5第二个旋转 8爪子开合
    ui->arm1_updown_lineedit->setText(QString::number(ptr.position[1]));
    ui->left_right_lineedit->setText(QString::number(ptr.position[0]));
    ui->arm2_updown_lineedit->setText(QString::number(ptr.position[2]));
    ui->paw_leftright_lineddit->setText(QString::number(ptr.position[10]));
    ui->paw_rotate_lineedit->setText(QString::number(ptr.position[16]));
    ui->paw_updown_lineedit->setText(QString::number(ptr.position[9]));
}

//*******get the message from the slider *****************
//get the speed from the slider and send the speed 
//it will run all the time if there is not a zero be send !!!!!!!!!!!!!!!!

//wheel
void explorer_ui_control::ui_wheel_left_speed_change(int value){
    ui->left_right_wheel_cmd_lineedit->setText(QString::number(ui->left_right_wheel_slider->value()*0.015));

    vel_publisher.angular.z = value*0.015;
    //explorer_ui_control::wheel_speed_pub(vel_publisher);
    ui_wheel_slider_pub.publish(vel_publisher);
    //vel_publisher.angular.z = 0.0;

}
void explorer_ui_control::ui_wheel_up_speed_change(int value){
    ui->up_down_wheel_cmd_lineedit->setText(QString::number(ui->up_down_wheel_slider->value()*0.015));
    vel_publisher.linear.x = value*0.015;
    //explorer_ui_control::wheel_speed_pub(vel_publisher);
    ui_wheel_slider_pub.publish(vel_publisher);
    //vel_publisher.linear.x = 0.0;
}


//vice wheel
//change the value
void explorer_ui_control::ui_up_left_vice_wheel_change(int value){
    ui->up_left_vice_wheel_cmd_lineedit->setText(QString::number(ui->up_left_vice_wheel_cmd_slider->value()));
    vice_vel_publisher.front_left_wheel_angular =value;
    ui_vice_wheel_slider_pub.publish(vice_vel_publisher);

}
void explorer_ui_control::ui_up_right_vice_wheel_change(int value){
    ui->up_right_vice_wheel_cmd_lineedit->setText(QString::number(ui->up_right_vice_wheel_cmd_slider->value()));
    vice_vel_publisher.front_right_wheel_angular =value;
    ui_vice_wheel_slider_pub.publish(vice_vel_publisher);

}
void explorer_ui_control::ui_down_left_vice_wheel_change(int value){
    ui->down_left_vice_wheel_cmd_lineedit->setText(QString::number(ui->down_left_vice_wheel_cmd_slider->value()));
    vice_vel_publisher.back_left_wheel_angular =value;
    ui_vice_wheel_slider_pub.publish(vice_vel_publisher);

}
void explorer_ui_control::ui_down_right_vice_wheel_change(int value){
    ui->down_right_vice_wheel_cmd_lineedit->setText(QString::number(ui->down_right_vice_wheel_cmd_slider->value()));
    vice_vel_publisher.back_right_wheel_angular =value;
    ui_vice_wheel_slider_pub.publish(vice_vel_publisher);

}
//********show all the states to the lineEdit*************



/***********************************


*/
//系统调用 可以用来在点击的使用启动launch文件
/*
void MainWindow::on_pushButton_clicked()
{
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;
             roslaunch ur_gazebo ur5.launch limited:=true'&");
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;
             roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true'&");
    system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;
             rosrun rviz rviz'&");
    exit(0);
}
*/
void explorer_ui_control::on_check_clicked()
{
    //system("source ~/catkin_ws/devel/setup.bash;"
           //"rosrun rviz rviz");
}

