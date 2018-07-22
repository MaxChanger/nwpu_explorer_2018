#ifndef VICTIM_CHOOSE_H
#define VICTIM_CHOOSE_H

#include <QWidget>
#include <QMouseEvent>
#include <QPushButton>

namespace Ui {
class VictimChoose;
}

class VictimChoose : public QWidget
{
    Q_OBJECT

public:
    explicit VictimChoose(QWidget *parent = 0);
    ~VictimChoose();

private:
    Ui::VictimChoose *ui;

private:
    void mouseMoveEvent(QMouseEvent *event);


signals:
    void sendVictim(int id) ;

private slots:
    void addActivityMark() ;
    void addInjuryMark() ;
    void addNoIndictionMark();
    void addNoMark();
private:
    bool pointInButton(QPoint current_mouse_point ,QPushButton *currnet_button) ;

    void addActivityExplain() ;
    void addInjuryExpain() ;
    void addNoIndictionExplain() ;
    void addNoMarkExplain() ;
};

#endif // VICTIM_CHOOSE_H
