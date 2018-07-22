#include "explorer_app_new/victim_choose.h"
#include "explorer_app_new/ui_victim_choose.h"
#include <QDebug>

VictimChoose::VictimChoose(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VictimChoose)
{
    ui->setupUi(this);

    this->setMouseTracking(true);
    this->setWindowFlags(Qt::WindowMinimizeButtonHint|Qt::CustomizeWindowHint);
    ui->pushButton_activity_button->setMouseTracking(true);
    ui->pushButton_injury_button->setMouseTracking(true);
    ui->pushButton_no_indiction_button->setMouseTracking(true);
    ui->pushButton_no_mark_button->setMouseTracking(true);
    ui->textEdit_victim_info_textEdit->setEnabled(false);

    connect(ui->pushButton_activity_button,SIGNAL(clicked()),this,SLOT(addActivityMark())) ;
    connect(ui->pushButton_injury_button,SIGNAL(clicked()),this,SLOT(addInjuryMark())) ;
    connect(ui->pushButton_no_indiction_button,SIGNAL(clicked()),this,SLOT(addNoIndictionMark())) ;
    connect(ui->pushButton_no_mark_button,SIGNAL(clicked()),this,SLOT(addNoMark())) ;
}

VictimChoose::~VictimChoose()
{
    delete ui;
}

bool VictimChoose::pointInButton(QPoint current_mouse_point, QPushButton *currnet_button)
{
    int height = currnet_button->height() ;
    int width = currnet_button->width() ;
    QPoint current_button_min_pose = currnet_button->pos() ;
    QPoint current_button_max_pose = currnet_button->pos() ;
    current_button_max_pose.setX(currnet_button->pos().x()+width);
    current_button_max_pose.setY(currnet_button->pos().y()+height);
    if((current_mouse_point.x()>=current_button_min_pose.x()&&current_mouse_point.y()>=current_button_min_pose.y())&&(
            current_mouse_point.x()<=current_button_max_pose.x()&&current_mouse_point.y()<=current_button_max_pose.y()))
    {
        return true ;
    }
    else
    {
        return false ;
    }
}

void VictimChoose::mouseMoveEvent(QMouseEvent *event)
{
    event->accept();
    QPoint current_mouse_point = event->pos() ;


    if(pointInButton(current_mouse_point,ui->pushButton_activity_button))
    {
        ui->textEdit_victim_info_textEdit->clear();
        addActivityExplain();
    }
    else if(pointInButton(current_mouse_point,ui->pushButton_injury_button))
    {

        ui->textEdit_victim_info_textEdit->clear();
        addInjuryExpain();
    }
    else if(pointInButton(current_mouse_point,ui->pushButton_no_indiction_button))
    {
        ui->textEdit_victim_info_textEdit->clear();
        addNoIndictionExplain();
    }
    else if(pointInButton(current_mouse_point,ui->pushButton_no_mark_button))
    {
        ui->textEdit_victim_info_textEdit->clear();
        addNoMarkExplain();
    }
    else
    {
        ui->textEdit_victim_info_textEdit->clear();
    }
}

void VictimChoose::addActivityExplain()
{
    ui->textEdit_victim_info_textEdit->append("The victim is in good condition or just some light wound ,  a <font color='green'><b> green </b></font>point  will be marked on the map");
}

void VictimChoose::addInjuryExpain()
{
    ui->textEdit_victim_info_textEdit->append("The victim is in bad condition need some emergency treatment, a <font color='red'><b> red </b></font>point  will be marked on the map");
}

void VictimChoose::addNoIndictionExplain()
{
    ui->textEdit_victim_info_textEdit->append("The victim is dead , a <font color='yellow'><b> yellow </b></font>point  will be marked on the map");
}

void VictimChoose::addNoMarkExplain()
{
    ui->textEdit_victim_info_textEdit->append("Maybe the camera get some error , and no victim near the robot,no point will be marked on the map");
}

void VictimChoose::addActivityMark()
{
    emit sendVictim(0);
    this->close() ;
}

void VictimChoose::addInjuryMark()
{
    emit sendVictim(1);
    this->close() ;
}

void VictimChoose::addNoIndictionMark()
{
    emit sendVictim(2);
    this->close() ;
}

void VictimChoose::addNoMark()
{
    emit sendVictim(3);
    this->close() ;
}
