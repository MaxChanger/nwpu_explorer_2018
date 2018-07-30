#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qterminal.h"
#include "qroscam.h"
#include "explorer_ui_control.h"
#include "explorer_ui_ability.h"
#include "qexplorerserver.h"
#include "pingtest.h"

#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(tr("explorer mainwindow"));
    setWindowIcon(QIcon(":/explorer_icon/image/explorer.jpg"));
    bash_command = "terminator -x bash -c ";

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_terminal_clicked()
{
    PingTest *pingtest = new PingTest;
    pingtest->show();
}

void MainWindow::on_diff_nocam_clicked()
{
    explorer_ui_control *ui = new explorer_ui_control();
    ui->show();
    system("terminator -x bash -c 'source ~/catkin_ws/devel/setup.bash;"
        "roslaunch explorer_launch start_explorer_client.launch limited:=true'&");
//    QTerminal *qterminal = new QTerminal(explorer_usrn, explorer_ipad);
//    qterminal->show();
//    qterminal->ROSProcess->write("cd catkin_ws \n");
//    qterminal->ROSProcess->write("roslaunch explorer_launch start_explorer_server.launch \n"); //QTerminal功能尚未完善 暂不使用
}

void MainWindow::on_diff_withcam_clicked()
{
    //摄像头的显示在另外加上去

    explorer_ui_control *ui = new explorer_ui_control();
    ui->show();
    system("terminator -x bash -c 'source ~/catkin_ws/devel/setup.bash;"
        "roslaunch explorer_launch start_explorer_client.launch limited:=true'&");//开一个新终端运行在工控机上并运行launch文件
//    QTerminal *qterminal = new QTerminal(explorer_usrn, explorer_ipad);
//    qterminal->show();
//    qterminal->ROSProcess->write("cd catkin_ws \n");
//    qterminal->ROSProcess->write("roslaunch explorer_launch start_explorer_server.launch \n"); //QTerminal功能尚未完善 暂不使用

}

void MainWindow::on_ablty_btn_clicked()
{
    //能力检测
    //底盘+机械臂+摄像头 
    //底盘机械臂不加控制 只有显示状态

    explorer_ui_ability *ui = new explorer_ui_ability();
    ui->show();
}

void MainWindow::on_arm_opt_btn_clicked()
{
    //arm的加上摄像头以及机械臂状态数据
    //
}

void MainWindow::on_CamButton_clicked()
{
    QRoscam *qroscam = new QRoscam(explorer_usrn, explorer_ipad);
    qroscam->show();
}

void MainWindow::on_serverButton_clicked()
{
    QExplorerserver *qexplorerserver = new QExplorerserver(explorer_usrn, explorer_ipad);
    qexplorerserver->show();
}
