#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "light_warn.h"
#include <explorer_app_new/victim_choose.h>
#include <vector>

using namespace std;

namespace  Ui {
class Widget;
}

class Widget:public QWidget
{
    Q_OBJECT

   public:
    explicit Widget(QWidget *parent = 0);
    void getInit();
    ~Widget();

public:
    int widget_width_;
    int widget_height_;

    LightWarn *qr_label_warn__ ;
    LightWarn *victim_label_warn__ ;
    LightWarn *victim_button_warn__ ;
    LightWarn *qr_button_warn__ ;

    vector<QString> image_frame_sub_name_ ;
    VictimChoose *victim_choose_widget ;

    void changeRobotLight(QLabel *label,bool light_info) ;
signals:
    void closeLabelFlicker() ;
    void closeButtonFlicker() ;
public slots:
    void victim_chooseOpen() ;
    void qr_codeMessageBox() ;

    //void on_pushButton_full_screen_button_clicked();


private:
    Ui::Widget *ui ;
};

#endif //WIDGET_H
