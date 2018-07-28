#ifndef QTERMINAL_H
#define QTERMINAL_H

#ifndef Q_MOC_RUN

#include <csignal>
#include <unistd.h>

#endif

#include <QWidget>
#include <QProcess>
#include <QDebug>
#include <QKeyEvent>
#include <QScrollBar>

namespace Ui {
class QTerminal;
}

class QTerminal : public QWidget
{
    Q_OBJECT

public:
    explicit QTerminal(QString usrn, QString ipad, QWidget *parent = 0);
    ~QTerminal();

    QProcess *ROSProcess;

    QString RegistPro;
    QString ProcesOutput;

    static void SignalHandle(int signum);

private Q_SLOTS:
    void on_commandButton_clicked();
    void keyPressEvent(QKeyEvent * event);
    void ReadOutput();

    void on_cancelButton_clicked();

private:
    Ui::QTerminal *ui;
};

#endif // QTERMINAL_H
