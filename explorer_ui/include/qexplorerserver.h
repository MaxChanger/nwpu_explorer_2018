#ifndef QEXPLORERSERVER_H
#define QEXPLORERSERVER_H

#ifndef Q_MOC_RUN

#include <iostream>
#include <stdio.h>
#include <string>

#endif

#include <QMainWindow>
#include <QSettings>
#include <QCheckBox>

namespace Ui {
class QExplorerserver;
}

class QExplorerserver : public QMainWindow
{
    Q_OBJECT

public:
    explicit QExplorerserver(QString explorer_usrn, QString explorer_ipad, QWidget *parent = 0);
    ~QExplorerserver();
    QString CommandGen(QString Command);
    void ReadSettings();
//    void WriteSettings();

private Q_SLOTS:
    void on_geotiffButton_clicked();
    void on_objtrackButton_clicked();
    void on_qrcodeButton_clicked();
    void on_camStartButton_clicked();
    void on_hectorConButton_clicked();
    void on_plannerButton_clicked();
    void on_slamButton_clicked();
    void on_rplidarButton_clicked();
    void on_serverButton_clicked();

    void WriteSettings();

    void on_saveButton_clicked();

private:
    Ui::QExplorerserver *ui;
    QString bash_command;
    QString usrn;
    QString ipad;
    QString source_ros;
    QString source_ws;

    QString serverCommand;
    QString rplidarCommand;
    QString slamCommand;
    QString plannerCommand;
    QString hectorCommand;
    QString testCamCommand;
    QString qrcodeCommand;
    QString objtrackCommand;
    QString geotiffCommand;

};

#endif // QEXPLORERSERVER_H
