#include "mainwindow.h"
#include "login.h"
#include "qterminal.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Login l;
    MainWindow w;
    l.show();

    if (l.exec() == QDialog::Accepted)
    {
        w.explorer_usrn = l.usrn;
        w.explorer_ipad = l.ipad;
        w.show();
        return a.exec();
    }
    else return 0;
}
