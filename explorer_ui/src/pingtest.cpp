#include "pingtest.h"
#include "ui_pingtest.h"
#include "qterminal.h"

PingTest::PingTest(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PingTest)
{
    ui->setupUi(this);
    ui->explorerlineEdit->setText("192.168.188.123");
    ui->localBridgelineEdit->setText("192.168.188.253");
    ui->remoteBridgelineEdit->setText("192.168.188.254");
}

PingTest::~PingTest()
{
    delete ui;
}

void PingTest::on_ping123Button_clicked()
{
    QString explorer_usrn, explorer_ipad;
    explorer_ipad = ui->explorerlineEdit->text();
    QTerminal *qterminal = new QTerminal(explorer_usrn, explorer_ipad);
    qterminal->show();
}

void PingTest::on_ping253Button_clicked()
{
    QString explorer_usrn, explorer_ipad;
    explorer_ipad = ui->localBridgelineEdit->text();
    QTerminal *qterminal = new QTerminal(explorer_usrn, explorer_ipad);
    qterminal->show();
}

void PingTest::on_ping254Button_clicked()
{
    QString explorer_usrn, explorer_ipad;
    explorer_ipad = ui->remoteBridgelineEdit->text();
    QTerminal *qterminal = new QTerminal(explorer_usrn, explorer_ipad);
    qterminal->show();
}
