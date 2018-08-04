#include "qexplorerserver.h"
#include "ui_qexplorerserver.h"
#include <QDebug>

QExplorerserver::QExplorerserver(QString explorer_usrn, QString explorer_ipad, QWidget *parent) :
    QMainWindow(parent),usrn(explorer_usrn), ipad(explorer_ipad),
    ui(new Ui::QExplorerserver)
{
    ui->setupUi(this);
    setWindowTitle(tr("explorer server"));
    ReadSettings();
    bash_command = "terminator -x bash -c ";
    source_ros = "source /opt/ros/kinetic/setup.bash;";
    source_ws = "source ~/catkin_ws/devel/setup.bash; source /home/explorer/catkin_cartographer_0.3.0/install_isolated/setup.bash; ";

    ui->serverEdit->setText(serverCommand);
    ui->rplidarEdit->setText(rplidarCommand);
    ui->slamEdit->setText(slamCommand);
    ui->plannerEdit->setText(plannerCommand);
    ui->hectorConEdit->setText(hectorCommand);
    ui->camStartEdit->setText(testCamCommand);
    ui->qrcodeEdit->setText(qrcodeCommand);
    ui->objtrackEdit->setText(objtrackCommand);
    ui->geotiffEdit->setText(geotiffCommand);

}

QExplorerserver::~QExplorerserver()
{
    delete ui;
}

void QExplorerserver::ReadSettings()
{
    QSettings settings("explorer_qt_server", "by explorer 2016");
    restoreGeometry(settings.value("geometry").toByteArray());
    serverCommand   = settings.value("serverCommand"    , "roslaunch explorer_launch start_explorer_server.launch").toString();
    rplidarCommand  = settings.value("rplidarCommand"   , "roslaunch rplidar_ros rplidar.launch").toString();
    slamCommand     = settings.value("slamCommand"      , "roslaunch cartographer_ros explorer_slam").toString();
    plannerCommand  = settings.value("plannerCommand"   , "roslaunch hector_exploration_node exploration_planner.launch").toString();
    hectorCommand   = settings.value("hectorCommand"    , "rosrun hector_exploration_controller simple_exploration_controller").toString();
    testCamCommand  = settings.value("testCamCommand"   , "roslaunch explorer_launch two_test.launch").toString();
    qrcodeCommand   = settings.value("qrcodeCommand"    , "roslaunch explorer_qrcode qrcode_detection.launch").toString();
    objtrackCommand = settings.value("objtrackCommand"  , "roslaunch hector_object_tracker camera_look_at.launch").toString();
    geotiffCommand  = settings.value("geotiffCommand"   , "roslaunch hector_geotiff geotiff_mapper_two.launch").toString();

}

void QExplorerserver::WriteSettings()
{
    QSettings settings("explorer_qt_server", "by explorer 2016");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("serverCommand", ui->serverEdit->text());
    settings.setValue("rplidarCommand", ui->rplidarEdit->text());
    settings.setValue("slamCommand", ui->slamEdit->text());
    settings.setValue("plannerCommand", ui->plannerEdit->text());
    settings.setValue("hectorCommand", ui->hectorConEdit->text());
    settings.setValue("testCamCommand", ui->camStartEdit->text());
    settings.setValue("qrcodeCommand", ui->qrcodeEdit->text());
    settings.setValue("objtrackCommand", ui->objtrackEdit->text());
    settings.setValue("geotiffCommand", ui->geotiffEdit->text());
}

QString QExplorerserver::CommandGen(QString Command)
{
    if(usrn.size() && ipad.size()) return (bash_command+"\"ssh -tt "+usrn+"@"+ipad+ " '"+source_ros+source_ws+Command+"; tail -f /dev/null'\"");
    else return (bash_command + " '" + Command + "; tail -f /dev/null '");
    //ssh远程登录命令格式为terminator -x bash -c "ssh -tt usrn@ipad 'source ${SOURCE_ROS} ${SOURCE_WS}; ros命令; tail -f /dev/null'"
    //terminator -x bash -c为开启terminator模拟器进入bash,并运行ssh
    //tail -f /dev/null会在命令运行完以后进入一个停滞状态，此时可以读取命令行输出
    /*ssh命令中，-tt参数为强制分配伪终端，不加参数会导致终端无返回值;需要source ros文件夹和工作空间文件夹的原因是因为ssh登录直接运行命令
    为non-login & non-interactive模式shell，只加载${BASH_ENV}变量而该变量为空而只能手动加载ros环境变量*/
}

void QExplorerserver::on_serverButton_clicked()
{
    QString ros_command;
    serverCommand = ui->serverEdit->text();
    ros_command = CommandGen(serverCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_rplidarButton_clicked()
{
    QString ros_command;
    rplidarCommand = ui->rplidarEdit->text();
    ros_command = CommandGen(rplidarCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_slamButton_clicked()
{
    QString ros_command;
    slamCommand = ui->slamEdit->text();
    ros_command = CommandGen(slamCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_plannerButton_clicked()
{
    QString ros_command;
    plannerCommand = ui->plannerEdit->text();
    ros_command = CommandGen(plannerCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_hectorConButton_clicked()
{
    QString ros_command;
    hectorCommand = ui->hectorConEdit->text();
    ros_command = CommandGen(hectorCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_camStartButton_clicked()
{
    QString ros_command;
    testCamCommand = ui->camStartEdit->text();
    ros_command = CommandGen(testCamCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_qrcodeButton_clicked()
{
    QString ros_command;
    qrcodeCommand = ui->qrcodeEdit->text();
    ros_command = bash_command + " '" + qrcodeCommand + "; tail -f /dev/null '";//qrcode在本地运行
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_objtrackButton_clicked()
{
    QString ros_command;
    objtrackCommand = ui->objtrackEdit->text();
    ros_command = CommandGen(objtrackCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_geotiffButton_clicked()
{
    QString ros_command;
    geotiffCommand = ui->geotiffEdit->text();
    ros_command = CommandGen(geotiffCommand);
    std::string command_string = ros_command.toStdString();
    popen(command_string.c_str(), "r");
}

void QExplorerserver::on_saveButton_clicked()
{
    WriteSettings();
}
