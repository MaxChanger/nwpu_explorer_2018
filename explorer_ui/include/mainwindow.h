#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QString explorer_usrn;
    QString explorer_ipad;

private Q_SLOTS:

    void on_terminal_clicked();

    void on_diff_nocam_clicked();

    void on_diff_withcam_clicked();
    void on_ablty_btn_clicked();
    void on_arm_opt_btn_clicked();

    void on_CamButton_clicked();

    void on_serverButton_clicked();

private:
    Ui::MainWindow *ui;
    QString bash_command;
};

#endif // MAINWINDOW_H
