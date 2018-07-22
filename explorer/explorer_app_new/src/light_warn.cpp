#include "explorer_app_new/light_warn.h"

LightWarn::LightWarn(QObject *parent) :
    QObject(parent)
{

}

void LightWarn::flickerOver()
{
    this->stopFlicker();
}

LabelWarn::LabelWarn(QLabel *label, QString original_icon, QString target_icon, QObject *parent):
    mainpulator_label__(label),
        label_original_icon__(original_icon),
        label_target_icon__(target_icon),
        timeout_times__(0)
{
    widget_flicker_ = false ;
     time_clock__ = new QTimer() ;
    time_clock__->setSingleShot(false);
    time_clock__->setInterval(500);
    connect(time_clock__ , SIGNAL(timeout()),this , SLOT(labelFlicker())) ;
}

LabelWarn::~LabelWarn()
{

}

void LabelWarn::widgetFlicker()
{
    timeout_times__ = 0 ;
    widget_flicker_ = true ;
    this->time_clock__->start();
}

void LabelWarn::stopFlicker()
{
    this->time_clock__->stop() ;
    widget_flicker_ = false ;
    mainpulator_label__->setPixmap(QPixmap(label_original_icon__));
}

void LabelWarn::labelFlicker()
{
   if(0==timeout_times__%2)
   {
       mainpulator_label__->setPixmap(QPixmap(label_original_icon__));
   }
   else {
       mainpulator_label__->setPixmap(QPixmap(label_target_icon__));
   }
   timeout_times__++ ;
}

ButtonWarn::ButtonWarn(QPushButton *button, QString font_info, QObject *parent):
    mainpulator_button__(button),
    timeout_times__(0),
    font_info__(font_info)
{
    widget_flicker_ = false ;
    time_clock__ = new QTimer() ;
    time_clock__->setSingleShot(false);
    time_clock__->setInterval(500);
    connect(time_clock__ , SIGNAL(timeout()),this , SLOT(buttonFlicker())) ;
}

ButtonWarn::~ButtonWarn()
{

}

void ButtonWarn::widgetFlicker()
{
    timeout_times__ = 0 ;
    widget_flicker_ = true ;
    time_clock__->start();
}

void ButtonWarn::stopFlicker()
{
    time_clock__->stop();
    widget_flicker_ = false ;
    mainpulator_button__->setStyleSheet(" "+font_info__);
}

void ButtonWarn::buttonFlicker()
{
    if(0==timeout_times__%2)
    {
        mainpulator_button__->setStyleSheet("QPushButton { background-color: rgb(255, 255, 255); }\n"+font_info__);
    }
    else {
        mainpulator_button__->setStyleSheet("QPushButton { background-color: rgb(255, 10, 10); }\n"+font_info__);
    }
    timeout_times__++ ;
}
