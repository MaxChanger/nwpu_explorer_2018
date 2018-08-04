#include "login.h"
#include "ui_login.h"

Login::Login(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Login)
{
    ui->setupUi(this);
    setWindowTitle(tr("explorer startup"));
    ReadSettings();
    setWindowIcon(QIcon(":/explorer_icon/image/explorer.jpg"));

    ui->loginButton->setDefault(true);
    ui->loginButton->setEnabled(true);

    connect(ui->usernameEdit, SIGNAL(textChanged(QString)),
            this, SLOT(enableloginButton()));
    connect(ui->ipEdit, SIGNAL(textChanged(QString)),
            this, SLOT(enableloginButton()));
}

Login::~Login()
{
    delete ui;
}

void Login::ReadSettings() {
    QSettings settings("explorer_qt_startup", "by explorer 2016");
    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
    usrn = settings.value("usrn", "explorer").toString();
    ipad = settings.value("ipad", "192.168.188.123").toString();

    ui->usernameEdit->setText(usrn);
    ui->ipEdit->setText(ipad);

    bool remember = settings.value("is_remembered", true).toBool();
    ui->rememberCheck->setChecked(remember);

}


void Login::on_loginButton_clicked()
{
  usrn = ui->usernameEdit->text();
  ipad = ui->ipEdit->text();
 // ui->ipEdit->setText(usern);
  //ui->passlabel->setText(ipad);

  if(ui->usernameEdit->text() == tr("explorer")&& ipad == tr("192.168.188.123"))
  {
        enterControlInterface();
        accept();
  }
  else if(ui->usernameEdit->text() == tr("root")&& ipad == tr("107.170.206.42"))
  {
      enterControlInterface();
      accept();
  }
  else if(ui->usernameEdit->text().isEmpty() && ui->ipEdit->text().isEmpty())
  {
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, tr("Local test mode"), tr("You will enter local test mode, sure?"), QMessageBox::No | QMessageBox::Yes);
        if(reply == QMessageBox::Yes) {enterControlInterface();accept();}
  }
  else
  {
      QMessageBox::StandardButton reply;
      reply = QMessageBox::question(this,tr("Not default user name"),tr("It is not default explorer name and ipaddress, continue or not?"), QMessageBox::Yes | QMessageBox::No);
      if(reply == QMessageBox::Yes)
      {
              enterControlInterface();
              accept();
      }
  }
}

void Login::on_closeButton_clicked()
{
    close();
}

void Login::enableloginButton()
{
   bool enable(!ui->usernameEdit->text().isEmpty() && !ui->ipEdit->text().isEmpty());
}

void Login::enterControlInterface()
{
  // robotinterface.show();
   //  this->hide();
    WriteSettings();
     close();
}

void Login::WriteSettings()
{
    QSettings settings("explorer_qt_startup", "by explorer 2016");
    if(ui->rememberCheck->isChecked())
    {
        settings.setValue("geometry", saveGeometry());
        settings.setValue("usrn", ui->usernameEdit->text());
        settings.setValue("ipad", ui->ipEdit->text());
        settings.setValue("is_remembered", true);
    }
    else settings.setValue("is_remembered", false);
}
