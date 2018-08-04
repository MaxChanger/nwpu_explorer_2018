#ifndef QROSCAM_H
#define QROSCAM_H

#include <QWidget>
#include <QMainWindow>
#include <QProcess>
#include <QSettings>
#include "qusbcam.h"

using namespace qusbcam;

namespace Ui {
class QRoscam;
}

class QRoscam : public QMainWindow
{
    Q_OBJECT

public:
    explicit QRoscam(QString usrn, QString ipad, QWidget *parent = 0);
    ~QRoscam();

    QUsbcam qusbcam;
    void ReadSettings();
    void WriteSettings();

private Q_SLOTS:
    void on_rqtButton_clicked();

    void on_video0Button_clicked();

    void on_video1Button_clicked();

    void on_video2Button_clicked();

    void on_video3Button_clicked();

    void on_video4Button_clicked();

//    void on_launchButton_clicked();

    void on_infraButton_clicked();

    void on_video5Button_clicked();

    void on_customButton_clicked();

    void on_saveButton_clicked();

private:
    Ui::QRoscam *ui;

    QString bash_command;
    QString usrn;
    QString ipad;
    QString source_ros;
    QString source_ws;
};

#endif // QROSCAM_H
