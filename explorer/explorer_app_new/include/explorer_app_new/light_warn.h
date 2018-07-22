#ifndef LIGHT_WARN_H
#define LIGHT_WARN_H

#include <QObject>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

class LightWarn : public QObject
{
    Q_OBJECT
public:
    explicit LightWarn(QObject *parent = 0);
    virtual void widgetFlicker() = 0;
    virtual void stopFlicker() = 0;
    bool widget_flicker_;
 protected:
 signals:

  public slots:
      void flickerOver();
};

class LabelWarn : public LightWarn
{
    Q_OBJECT
public:
    explicit LabelWarn(QLabel *label , QString original_icon,QString target_icon,QObject *parent = 0) ;
       ~LabelWarn() ;
    void widgetFlicker();
    void stopFlicker();

private slots:
    void labelFlicker();

private:
    QLabel *mainpulator_label__;
    QString label_original_icon__;
    QString label_target_icon__;
    QTimer *time_clock__;
    int timeout_times__;
};

class ButtonWarn : public LightWarn
{
    Q_OBJECT
 public:
    explicit ButtonWarn(QPushButton *button,QString font_info = NULL,QObject *parent = 0);
    ~ButtonWarn();
    void widgetFlicker();
    void stopFlicker();

 private slots:
    void buttonFlicker();

 private:
    QPushButton *mainpulator_button__;
    QTimer *time_clock__;
    QString font_info__;
    int timeout_times__;
};

#endif //LIGHT_WARN_H
