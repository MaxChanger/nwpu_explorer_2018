/********************************************************************************
** Form generated from reading UI file 'victim_choose.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VICTIM_CHOOSE_H
#define UI_VICTIM_CHOOSE_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QTextEdit>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_VictimChoose
{
public:
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QPushButton *pushButton_activity_button;
    QTextEdit *textEdit_victim_info_textEdit;
    QPushButton *pushButton_injury_button;
    QPushButton *pushButton_no_indiction_button;
    QPushButton *pushButton_no_mark_button;

    void setupUi(QWidget *VictimChoose)
    {
        if (VictimChoose->objectName().isEmpty())
            VictimChoose->setObjectName(QString::fromUtf8("VictimChoose"));
        VictimChoose->resize(400, 300);
        layoutWidget = new QWidget(VictimChoose);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(0, 7, 391, 281));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_activity_button = new QPushButton(layoutWidget);
        pushButton_activity_button->setObjectName(QString::fromUtf8("pushButton_activity_button"));

        gridLayout->addWidget(pushButton_activity_button, 0, 0, 1, 1);

        textEdit_victim_info_textEdit = new QTextEdit(layoutWidget);
        textEdit_victim_info_textEdit->setObjectName(QString::fromUtf8("textEdit_victim_info_textEdit"));

        gridLayout->addWidget(textEdit_victim_info_textEdit, 0, 1, 4, 1);

        pushButton_injury_button = new QPushButton(layoutWidget);
        pushButton_injury_button->setObjectName(QString::fromUtf8("pushButton_injury_button"));

        gridLayout->addWidget(pushButton_injury_button, 1, 0, 1, 1);

        pushButton_no_indiction_button = new QPushButton(layoutWidget);
        pushButton_no_indiction_button->setObjectName(QString::fromUtf8("pushButton_no_indiction_button"));

        gridLayout->addWidget(pushButton_no_indiction_button, 2, 0, 1, 1);

        pushButton_no_mark_button = new QPushButton(layoutWidget);
        pushButton_no_mark_button->setObjectName(QString::fromUtf8("pushButton_no_mark_button"));

        gridLayout->addWidget(pushButton_no_mark_button, 3, 0, 1, 1);


        retranslateUi(VictimChoose);

        QMetaObject::connectSlotsByName(VictimChoose);
    } // setupUi

    void retranslateUi(QWidget *VictimChoose)
    {
        VictimChoose->setWindowTitle(QApplication::translate("VictimChoose", "Form", 0, QApplication::UnicodeUTF8));
        pushButton_activity_button->setText(QApplication::translate("VictimChoose", "Activity", 0, QApplication::UnicodeUTF8));
        pushButton_injury_button->setText(QApplication::translate("VictimChoose", "Injury", 0, QApplication::UnicodeUTF8));
        pushButton_no_indiction_button->setText(QApplication::translate("VictimChoose", "No Indiction", 0, QApplication::UnicodeUTF8));
        pushButton_no_mark_button->setText(QApplication::translate("VictimChoose", "No Mark", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class VictimChoose: public Ui_VictimChoose {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VICTIM_CHOOSE_H
