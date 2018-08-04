#include "qroscam.h"
#include "ui_qroscam.h"
#include "qusbcam.h"

QRoscam::QRoscam(QString explorer_usrn, QString explorer_ipad, QWidget *parent) :
    QMainWindow(parent), usrn(explorer_usrn), ipad(explorer_ipad),
    ui(new Ui::QRoscam)
{
    ui->setupUi(this);
    setWindowTitle(tr("roscam window"));
    ReadSettings();
    this->setCentralWidget(ui->centerWidget);//qmainwindow类有自己的布局管理器，需要新建一个widget作为中心widget才能设置布局
    bash_command = "terminator -x bash -c ";
    source_ros = "source /opt/ros/kinetic/setup.bash;";
    source_ws = "source ~/catkin_ws/devel/setup.bash; source /home/explorer/catkin_cartographer_0.3.0/install_isolated/setup.bash; ";
}

QRoscam::~QRoscam()
{
    delete ui;
}

void QRoscam::ReadSettings()
{
    QSettings settings("explorer_qroscam", "by explorer 2016");
    restoreGeometry(settings.value("geometry").toByteArray());
    ui->customEdit->setText(settings.value("customCommand", "rosrun usb_cam usb_cam_node").toString());
}

void QRoscam::WriteSettings()
{
    QSettings settings("explorer_qroscam", "by explorer 2016");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("customCommand", ui->customEdit->text());
}

void QRoscam::on_rqtButton_clicked()
{
    QProcess *rqtProcess = new QProcess;
    rqtProcess->start("rqt");
}

void QRoscam::on_video0Button_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch usb_cam usb_cam-video0.launch; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video0.launch; tail -f /dev/null'");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_video1Button_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch usb_cam usb_cam-video1.launch; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_video2Button_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch usb_cam usb_cam-video2.launch; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video2.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_video3Button_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch usb_cam usb_cam-video3.launch; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video3.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_video4Button_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch usb_cam usb_cam-video4.launch; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video4.launch; tail -f /dev/null '&");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(),"r");
}

//void QRoscam::on_launchButton_clicked()
//{
//    QString ros_command;
//    if(usrn.size() && ipad.size())
//    {
//        ros_command = (bash_command+"'ssh "+usrn+"@"+ipad+" 'roslaunch explorer_launch start_explorer_server.launch''&");
//    }
//    else
//    {
//        ros_command = (bash_command + " 'roslaunch explorer_launch start_explorer_server.launch'");
//    }
//    std::string command_string = ros_command.toStdString();
//    system(command_string.c_str());
//}

void QRoscam::on_infraButton_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t "+usrn+"@"+ipad+ " '"+source_ros+source_ws+"rosrun flir_one_node flir_one_node; tail -f /dev/null '\"");
        qDebug()<<ros_command;
    }
    else
    {
        ros_command = (bash_command + " 'rosrun flir_one_node flir_one_node; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_video5Button_clicked()
{
    QString ros_command;
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t"+usrn+"@"+ipad+ " '"+source_ros+source_ws+"roslaunch usb_cam usb_cam-video5.launch; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " 'roslaunch usb_cam usb_cam-video5.launch; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_customButton_clicked()
{
    QString ros_command;
    QString command = ui->customEdit->text();
    if(usrn.size() && ipad.size())
    {
        ros_command = (bash_command+"\"ssh -t"+usrn+"@"+ipad+ " '"+source_ros+source_ws+command+"; tail -f /dev/null '\"");
    }
    else
    {
        ros_command = (bash_command + " '"+command+"; tail -f /dev/null '");
    }
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QRoscam::on_saveButton_clicked()
{
    WriteSettings();
}
