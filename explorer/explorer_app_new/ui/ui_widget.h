/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Widget {
public:
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *qr_code_find_lable;
    QPushButton *qr_code_find_button;
    QPushButton *victim_find_button;
    QLabel *victim_find_lable;

    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *env_t_lable;
    QLCDNumber *env_t_num_show;
    QLabel *aim_t_lable;
    QLCDNumber *aim_t_num_show;

    QWidget *layoutWidget;
    QGridLayout *gridLayout_for_co2;
    QLabel *co2_lable;
    QSpacerItem *horizontalSpacer;
    QLCDNumber *co2_num_show;

    void setupUi(QWidget *Widget) {
        if (Widget->objectName().isEmpty()) {
            Widget->setObjectName(QString::fromUtf8("Widget"));
        }

        Widget->resize(570, 363);

        // 布局水平和垂直方向调整策略

        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);                                     // 设定垂直缩放因子
        sizePolicy.setVerticalStretch(0);                                       // 设定水平缩放因子
        sizePolicy.setHeightForWidth(Widget->sizePolicy().hasHeightForWidth()); // 表示部件的缺省大小

        Widget->setSizePolicy(sizePolicy);

        {
            // 设定次级框架
            gridLayoutWidget = new QWidget(Widget);
            gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
            gridLayoutWidget->setGeometry(QRect(9, 9, 147, 62));
            // 设定布局管理器
            gridLayout = new QGridLayout(gridLayoutWidget);
            gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
            gridLayout->setContentsMargins(0, 0, 0, 0);
            {

                // 设定标签
                victim_find_lable = new QLabel(gridLayoutWidget);
                victim_find_lable->setObjectName(QString::fromUtf8("victim_find_lable"));
                sizePolicy.setHeightForWidth(victim_find_lable->sizePolicy().hasHeightForWidth());
                victim_find_lable->setSizePolicy(sizePolicy);
                victim_find_lable->setPixmap(QPixmap(QString::fromUtf8(":/new/prefix1/no_info.png")));
                // 添加标签在第0行第0列横竖间距均为1
                gridLayout->addWidget(victim_find_lable, 0, 0, 1, 1);

                // 设定一些颜色
                // 黑色
                QBrush color_black(QColor(0, 0, 0, 255));
                color_black.setStyle(Qt::SolidPattern);
                //淡蓝
                QBrush color_light_blue(QColor(219, 255, 255, 255));
                color_light_blue.setStyle(Qt::SolidPattern);
                //白色
                QBrush color_white(QColor(255, 255, 255, 255));
                color_white.setStyle(Qt::SolidPattern);
                //淡蓝(淡)
                QBrush color_more_light_blue(QColor(237, 255, 255, 255));
                color_more_light_blue.setStyle(Qt::SolidPattern);
                //灰色
                QBrush color_gray(QColor(109, 127, 127, 255));
                color_gray.setStyle(Qt::SolidPattern);
                //灰色(淡)
                QBrush color_light_gray(QColor(146, 170, 170, 255));
                color_light_gray.setStyle(Qt::SolidPattern);
                //粉色
                QBrush color_pink(QColor(255, 255, 220, 255));
                color_pink.setStyle(Qt::SolidPattern);

                // 设置按钮
                victim_find_button = new QPushButton(gridLayoutWidget);
                victim_find_button->setObjectName(QString::fromUtf8("victim_find_button"));
                sizePolicy.setHeightForWidth(victim_find_button->sizePolicy().hasHeightForWidth());
                victim_find_button->setSizePolicy(sizePolicy);
                // 设置按钮颜色
                QPalette palette;
                palette.setBrush(QPalette::Active, QPalette::WindowText, color_black);
                palette.setBrush(QPalette::Active, QPalette::Button, color_light_blue);
                palette.setBrush(QPalette::Active, QPalette::Light, color_white);
                palette.setBrush(QPalette::Active, QPalette::Midlight, color_more_light_blue);
                palette.setBrush(QPalette::Active, QPalette::Dark, color_gray);
                palette.setBrush(QPalette::Active, QPalette::Mid, color_light_gray);
                palette.setBrush(QPalette::Active, QPalette::Text, color_black);
                palette.setBrush(QPalette::Active, QPalette::BrightText, color_white);
                palette.setBrush(QPalette::Active, QPalette::ButtonText, color_black);
                palette.setBrush(QPalette::Active, QPalette::Base, color_white);
                palette.setBrush(QPalette::Active, QPalette::Window, color_light_blue);
                palette.setBrush(QPalette::Active, QPalette::Shadow, color_black);
                palette.setBrush(QPalette::Active, QPalette::AlternateBase, color_more_light_blue);
                palette.setBrush(QPalette::Active, QPalette::ToolTipBase, color_pink);
                palette.setBrush(QPalette::Active, QPalette::ToolTipText, color_black);
                palette.setBrush(QPalette::Inactive, QPalette::WindowText, color_black);
                palette.setBrush(QPalette::Inactive, QPalette::Button, color_light_blue);
                palette.setBrush(QPalette::Inactive, QPalette::Light, color_white);
                palette.setBrush(QPalette::Inactive, QPalette::Midlight, color_more_light_blue);
                palette.setBrush(QPalette::Inactive, QPalette::Dark, color_gray);
                palette.setBrush(QPalette::Inactive, QPalette::Mid, color_light_gray);
                palette.setBrush(QPalette::Inactive, QPalette::Text, color_black);
                palette.setBrush(QPalette::Inactive, QPalette::BrightText, color_white);
                palette.setBrush(QPalette::Inactive, QPalette::ButtonText, color_black);
                palette.setBrush(QPalette::Inactive, QPalette::Base, color_white);
                palette.setBrush(QPalette::Inactive, QPalette::Window, color_light_blue);
                palette.setBrush(QPalette::Inactive, QPalette::Shadow, color_black);
                palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, color_more_light_blue);
                palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, color_pink);
                palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, color_black);
                palette.setBrush(QPalette::Disabled, QPalette::WindowText, color_gray);
                palette.setBrush(QPalette::Disabled, QPalette::Button, color_light_blue);
                palette.setBrush(QPalette::Disabled, QPalette::Light, color_white);
                palette.setBrush(QPalette::Disabled, QPalette::Midlight, color_more_light_blue);
                palette.setBrush(QPalette::Disabled, QPalette::Dark, color_gray);
                palette.setBrush(QPalette::Disabled, QPalette::Mid, color_light_gray);
                palette.setBrush(QPalette::Disabled, QPalette::Text, color_gray);
                palette.setBrush(QPalette::Disabled, QPalette::BrightText, color_white);
                palette.setBrush(QPalette::Disabled, QPalette::ButtonText, color_gray);
                palette.setBrush(QPalette::Disabled, QPalette::Base, color_light_blue);
                palette.setBrush(QPalette::Disabled, QPalette::Window, color_light_blue);
                palette.setBrush(QPalette::Disabled, QPalette::Shadow, color_black);
                palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, color_light_blue);
                palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, color_pink);
                palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, color_black);
                victim_find_button->setPalette(palette);
                // 设置按钮字体
                QFont font;
                font.setBold(true);
                font.setWeight(75);
                victim_find_button->setFont(font);
                // 添加标签在第0行第1列横竖间距均为1s
                gridLayout->addWidget(victim_find_button, 0, 1, 1, 1);

                qr_code_find_lable = new QLabel(gridLayoutWidget);
                qr_code_find_lable->setObjectName(QString::fromUtf8("qr_code_find_lable"));
                sizePolicy.setHeightForWidth(qr_code_find_lable->sizePolicy().hasHeightForWidth());
                qr_code_find_lable->setSizePolicy(sizePolicy);
                qr_code_find_lable->setPixmap(QPixmap(QString::fromUtf8(":/new/prefix1/no_info.png")));

                gridLayout->addWidget(qr_code_find_lable, 1, 0, 1, 1);

                qr_code_find_button = new QPushButton(gridLayoutWidget);
                qr_code_find_button->setObjectName(QString::fromUtf8("qr_code_find_button"));
                sizePolicy.setHeightForWidth(qr_code_find_button->sizePolicy().hasHeightForWidth());
                qr_code_find_button->setSizePolicy(sizePolicy);
                QPalette palette1;
                palette1.setBrush(QPalette::Active, QPalette::WindowText, color_black);
                palette1.setBrush(QPalette::Active, QPalette::Button, color_light_blue);
                palette1.setBrush(QPalette::Active, QPalette::Light, color_white);
                palette1.setBrush(QPalette::Active, QPalette::Midlight, color_more_light_blue);
                palette1.setBrush(QPalette::Active, QPalette::Dark, color_gray);
                palette1.setBrush(QPalette::Active, QPalette::Mid, color_light_gray);
                palette1.setBrush(QPalette::Active, QPalette::Text, color_black);
                palette1.setBrush(QPalette::Active, QPalette::BrightText, color_white);
                palette1.setBrush(QPalette::Active, QPalette::ButtonText, color_black);
                palette1.setBrush(QPalette::Active, QPalette::Base, color_white);
                palette1.setBrush(QPalette::Active, QPalette::Window, color_light_blue);
                palette1.setBrush(QPalette::Active, QPalette::Shadow, color_black);
                palette1.setBrush(QPalette::Active, QPalette::AlternateBase, color_more_light_blue);
                palette1.setBrush(QPalette::Active, QPalette::ToolTipBase, color_pink);
                palette1.setBrush(QPalette::Active, QPalette::ToolTipText, color_black);
                palette1.setBrush(QPalette::Inactive, QPalette::WindowText, color_black);
                palette1.setBrush(QPalette::Inactive, QPalette::Button, color_light_blue);
                palette1.setBrush(QPalette::Inactive, QPalette::Light, color_white);
                palette1.setBrush(QPalette::Inactive, QPalette::Midlight, color_more_light_blue);
                palette1.setBrush(QPalette::Inactive, QPalette::Dark, color_gray);
                palette1.setBrush(QPalette::Inactive, QPalette::Mid, color_light_gray);
                palette1.setBrush(QPalette::Inactive, QPalette::Text, color_black);
                palette1.setBrush(QPalette::Inactive, QPalette::BrightText, color_white);
                palette1.setBrush(QPalette::Inactive, QPalette::ButtonText, color_black);
                palette1.setBrush(QPalette::Inactive, QPalette::Base, color_white);
                palette1.setBrush(QPalette::Inactive, QPalette::Window, color_light_blue);
                palette1.setBrush(QPalette::Inactive, QPalette::Shadow, color_black);
                palette1.setBrush(QPalette::Inactive, QPalette::AlternateBase, color_more_light_blue);
                palette1.setBrush(QPalette::Inactive, QPalette::ToolTipBase, color_pink);
                palette1.setBrush(QPalette::Inactive, QPalette::ToolTipText, color_black);
                palette1.setBrush(QPalette::Disabled, QPalette::WindowText, color_gray);
                palette1.setBrush(QPalette::Disabled, QPalette::Button, color_light_blue);
                palette1.setBrush(QPalette::Disabled, QPalette::Light, color_white);
                palette1.setBrush(QPalette::Disabled, QPalette::Midlight, color_more_light_blue);
                palette1.setBrush(QPalette::Disabled, QPalette::Dark, color_gray);
                palette1.setBrush(QPalette::Disabled, QPalette::Mid, color_light_gray);
                palette1.setBrush(QPalette::Disabled, QPalette::Text, color_gray);
                palette1.setBrush(QPalette::Disabled, QPalette::BrightText, color_white);
                palette1.setBrush(QPalette::Disabled, QPalette::ButtonText, color_gray);
                palette1.setBrush(QPalette::Disabled, QPalette::Base, color_light_blue);
                palette1.setBrush(QPalette::Disabled, QPalette::Window, color_light_blue);
                palette1.setBrush(QPalette::Disabled, QPalette::Shadow, color_black);
                palette1.setBrush(QPalette::Disabled, QPalette::AlternateBase, color_light_blue);
                palette1.setBrush(QPalette::Disabled, QPalette::ToolTipBase, color_pink);
                palette1.setBrush(QPalette::Disabled, QPalette::ToolTipText, color_black);
                qr_code_find_button->setPalette(palette1);
                qr_code_find_button->setFont(font);
                gridLayout->addWidget(qr_code_find_button, 1, 1, 1, 1);
            }
        }
        {
            layoutWidget = new QWidget(Widget);
            layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
            layoutWidget->setGeometry(QRect(30, 80, 128, 111));

            gridLayout_for_co2 = new QGridLayout(layoutWidget);
            gridLayout_for_co2->setObjectName(QString::fromUtf8("gridLayout_for_co2"));
            gridLayout_for_co2->setContentsMargins(0, 0, 0, 0);
            {
                co2_lable = new QLabel(layoutWidget);
                co2_lable->setObjectName(QString::fromUtf8("co2_lable"));
                sizePolicy.setHeightForWidth(co2_lable->sizePolicy().hasHeightForWidth());
                co2_lable->setSizePolicy(sizePolicy);
                QFont font1;
                font1.setPointSize(12);
                font1.setBold(true);
                font1.setWeight(75);
                co2_lable->setFont(font1);
                gridLayout_for_co2->addWidget(co2_lable, 0, 0, 1, 1);

                horizontalSpacer = new QSpacerItem(88, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
                gridLayout_for_co2->addItem(horizontalSpacer, 0, 1, 1, 1);

                co2_num_show = new QLCDNumber(layoutWidget);
                co2_num_show->setObjectName(QString::fromUtf8("co2_num_show"));
                sizePolicy.setHeightForWidth(co2_num_show->sizePolicy().hasHeightForWidth());
                co2_num_show->setSizePolicy(sizePolicy);
                gridLayout_for_co2->addWidget(co2_num_show, 1, 0, 1, 2);
            }
        }
        {
            gridLayoutWidget_2 = new QWidget(Widget);
            gridLayoutWidget_2->setObjectName(QString::fromUtf8("gridLayoutWidget_2"));
            gridLayoutWidget_2->setGeometry(QRect(30, 80, 128, 111));

            gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
            gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
            gridLayout_2->setContentsMargins(0, 0, 0, 0);
            {
                env_t_lable = new QLabel(gridLayoutWidget_2);
                env_t_lable->setObjectName(QString::fromUtf8("env_t_lable"));
                sizePolicy.setHeightForWidth(env_t_lable->sizePolicy().hasHeightForWidth());
                env_t_lable->setSizePolicy(sizePolicy);
                QFont font1;
                font1.setPointSize(12);
                font1.setBold(true);
                font1.setWeight(75);
                env_t_lable->setFont(font1);
                gridLayout_2->addWidget(env_t_lable, 0, 0, 1, 1);

                env_t_num_show = new QLCDNumber(gridLayoutWidget_2);
                env_t_num_show->setObjectName(QString::fromUtf8("env_t_num_show"));
                sizePolicy.setHeightForWidth(env_t_num_show->sizePolicy().hasHeightForWidth());
                env_t_num_show->setSizePolicy(sizePolicy);
                gridLayout_2->addWidget(env_t_num_show, 0, 1, 1, 2);

                aim_t_lable = new QLabel(gridLayoutWidget_2);
                aim_t_lable->setObjectName(QString::fromUtf8("aim_t_lable"));
                sizePolicy.setHeightForWidth(aim_t_lable->sizePolicy().hasHeightForWidth());
                aim_t_lable->setSizePolicy(sizePolicy);
                QFont font1;
                font1.setPointSize(12);
                font1.setBold(true);
                font1.setWeight(75);
                aim_t_lable->setFont(font1);
                gridLayout_2->addWidget(aim_t_lable, 1, 0, 1, 1);

                aim_t_num_show = new QLCDNumber(gridLayoutWidget_2);
                aim_t_num_show->setObjectName(QString::fromUtf8("aim_t_num_show"));
                sizePolicy.setHeightForWidth(aim_t_num_show->sizePolicy().hasHeightForWidth());
                aim_t_num_show->setSizePolicy(sizePolicy);
                gridLayout_2->addWidget(aim_t_num_show, 1, 1, 1, 2);
            }
        }

        retranslateUi(Widget);

        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget) {
        Widget->setWindowTitle(QApplication::translate("Widget", "Form", 0, QApplication::UnicodeUTF8));
        victim_find_lable->setText(QString());
        victim_find_button->setText(QApplication::translate("Widget", "Victim Find", 0, QApplication::UnicodeUTF8));
        qr_code_find_lable->setText(QString());
        qr_code_find_button->setText(QApplication::translate("Widget", "QR Code Find", 0, QApplication::UnicodeUTF8));
        co2_lable->setText(QApplication::translate("Widget", "CO2", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H







