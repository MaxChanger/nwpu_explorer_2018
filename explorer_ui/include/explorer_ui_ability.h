#ifndef EXPLORER_UI_ABILITY_H
#define EXPLORER_UI_ABILITY_H

#include "explorer_thread.h"

#include <QMainWindow>

namespace Ui {
class explorer_ui_ability;
}

class explorer_ui_ability : public QMainWindow
{
    Q_OBJECT

public:
    explicit explorer_ui_ability(QWidget *parent = 0);
    ~explorer_ui_ability();
    

private:
    Ui::explorer_ui_ability *ui;
    explorer_thread *t;
private Q_SLOTS:
    void topic_co2(explorer_msgs::explorer_co2);

};

#endif // EXPLORER_UI_ABILITY_H
