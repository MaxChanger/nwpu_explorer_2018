#ifndef QEXPLORERSERVER_H
#define QEXPLORERSERVER_H

#ifndef Q_MOC_RUN //特别声明 防止对非qt头文件进行qt moc处理，否则会报错

#include <iostream>
#include <stdio.h>

#endif

#include <QMainWindow>

namespace Ui {
class QExplorerserver;
}

class QExplorerserver : public QMainWindow
{
    Q_OBJECT

public:
    explicit QExplorerserver(QString explorer_usrn, QString explorer_ipad, QWidget *parent = 0);
    ~QExplorerserver();

private Q_SLOTS: //关键字需要特别声明。如果出现关键字错误，尝试将Q_SLOTS改为slots
    void on_geotiffButton_clicked();
    void on_objtrackButton_clicked();
    void on_qrcodeButton_clicked();
    void on_camStartButton_clicked();
    void on_hectorConButton_clicked();
    void on_plannerButton_clicked();
    void on_slamButton_clicked();
    void on_rplidarButton_clicked();
    void on_serverButton_clicked();

private:
    Ui::QExplorerserver *ui;
    QString bash_command;
    QString usrn;
    QString ipad;
    QString source_ros;
    QString source_ws;
};

#endif // QEXPLORERSERVER_H
