#include "qexplorerserver.h"
#include "ui_qexplorerserver.h"
#include <QDebug>

QExplorerserver::QExplorerserver(QString explorer_usrn, QString explorer_ipad, QWidget *parent) :
    QMainWindow(parent),usrn(explorer_usrn), ipad(explorer_ipad),
    ui(new Ui::QExplorerserver)
{
    setWindowTitle(tr("explorer server"));
    ui->setupUi(this);
    bash_command = "terminator -x bash -c ";
    source_ros = "source /opt/ros/kinetic/setup.bash;";
    source_ws = "source ~/catkin_ws/devel/setup.bash; source /home/explorer/catkin_cartographer_0.3.0/install_isolated/setup.bash; ";
}

QExplorerserver::~QExplorerserver()
{
    delete ui;
}

void QExplorerserver::on_serverButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -tt "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch explorer_launch start_explorer_server.launch; tail -f /dev/null'\"");
//       ros_command = (bash_command + "\"ssh -t "+usrn+"@"+ipad+ " 'roscore; tail -f /dev/null'\"");
        qDebug()<<ros_command;
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch explorer_launch start_explorer_server.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_rplidarButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch rplidar_ros rplidar.launch; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch rplidar_ros rplidar.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_slamButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch cartographer_ros explorer_slam; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch cartographer_ros explorer_slam.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_plannerButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch hector_exploration_node exploration_planner.launch; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video0.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_hectorConButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"rosrun hector_exploration_controller simple_exploration_controller; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video0.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_camStartButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch explorer_launch two_test.launch; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video0.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_qrcodeButton_clicked()
{
    QString ros_command;
    ros_command = (bash_command + " 'roslaunch explorer_qrcode qrcode_detection.launch; tail -f /dev/null '");
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_objtrackButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch hector_object_tracker camera_look_at.launch; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch hector_object_tracker camera_look_at.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_geotiffButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch hector_geotiff geotiff_mapper_two.launch; tail -f /dev/null'\"");
    }
    else 
    {
        ros_command = (bash_command + " 'roslaunch hector_geotiff geotiff_mapper_two.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}
