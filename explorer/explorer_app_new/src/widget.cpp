#include "explorer_app_new/widget.h"
#include "explorer_app_new/ui_widget.h"
#include <QSplitter>
#include <QLCDNumber>
#include<QMessageBox>
#include <QDebug>

Widget::Widget(QWidget *parent):
    QWidget(parent) {

}

Widget::~Widget() {

}

void Widget::changeRobotLight(QLabel *label, bool light_info) {
    if (light_info == true) {
        QString pic_path(":/new/prefix1/lightOn.png");
        label->setPixmap(QPixmap(pic_path));
    } else {
        QString pic_path(":/new/prefix1/light_0.png");
        label->setPixmap(QPixmap(pic_path));
    }
}

void Widget::victim_chooseOpen() {
    if (!victim_choose_widget) {
        victim_choose_widget = new VictimChoose();
    }

    victim_choose_widget->move(0, 0);
    victim_choose_widget->show();
    qDebug() << "victim stop filcker";
    victim_label_warn__->stopFlicker();
    victim_button_warn__->stopFlicker();
}

void Widget::qr_codeMessageBox() {
    if (QMessageBox::question(this, "QR CODE", "Do you want to mark this qr code on the map?",
                              QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes) == QMessageBox::Yes) {
        qDebug() << "yes";
    } else {
        qDebug() << "no";
    }

    qDebug() << "qr code stop filcker";
    qr_label_warn__->stopFlicker();
    qr_button_warn__->stopFlicker();
}

//void Widget::on_pushButton_full_screen_button_clicked()
//{
//    this->showFullScreen();
//}