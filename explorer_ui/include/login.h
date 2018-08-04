#ifndef LOGIN_H
#define LOGIN_H

#include <QWidget>
#include <QtGui>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>


namespace Ui {
class Login;
}

class Login : public QDialog
{
    Q_OBJECT

public:
    explicit Login(QWidget *parent = 0);
    ~Login();
    void ReadSettings();
    void WriteSettings();

    QString usrn;
    QString ipad;

private Q_SLOTS:

    void on_loginButton_clicked();

    void on_closeButton_clicked();

    void enableloginButton();

  //  void on_usernameEdit_textChanged(const QString &arg1);

private:
    Ui::Login *ui;

    // MainInterface robotinterface;

    void enterControlInterface();
  // QLabel *passlabel;//
};

#endif // LOGIN_H
