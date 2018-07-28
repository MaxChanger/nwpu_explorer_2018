#ifndef PINGTEST_H
#define PINGTEST_H

#include <QWidget>

namespace Ui {
class PingTest;
}

class PingTest : public QWidget
{
    Q_OBJECT

public:
    explicit PingTest(QWidget *parent = 0);
    ~PingTest();

private Q_SLOTS:
    void on_ping254Button_clicked();

private Q_SLOTS:
    void on_ping253Button_clicked();

private Q_SLOTS:
    void on_ping123Button_clicked();

private:
    Ui::PingTest *ui;
    QString local_netbridge;
    QString remote_netbrige;
    QString explorer_;
};

#endif // PINGTEST_H
