#include "qterminal.h"
#include "ui_qterminal.h"

QTerminal::QTerminal(QString usrn, QString ipad, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QTerminal)
{
    ProcesOutput = "";
//    RegistPro = "ssh -t -t " + usrn + "@" + ipad;
    RegistPro = "ping "+ ipad;

    ui->setupUi(this);
    setWindowTitle("QTerminal");

    ROSProcess = new QProcess(this);

    connect(ROSProcess, SIGNAL(readyRead()), this, SLOT(ReadOutput()));
    ROSProcess->start(RegistPro);

}

QTerminal::~QTerminal()
{
    delete ui;
}

void QTerminal::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Return)
        on_commandButton_clicked();
    else if((event->key() == Qt::Key_C) && (event->modifiers() == Qt::ControlModifier))
    {
        on_cancelButton_clicked();
    }
}

void QTerminal::on_commandButton_clicked()
{

    ROSProcess->write(ui->commandEdit->text().toLatin1());
    QString key_return = "\n";
    ROSProcess->write(key_return.toLatin1());

    ui->commandEdit->setText("");
}

void QTerminal::ReadOutput()
{
    ProcesOutput = ROSProcess->readAll();
    ui->textEdit->append(ProcesOutput);
    QScrollBar *scrollbar = ui->textEdit->verticalScrollBar();
    if (scrollbar)
    {
        scrollbar->setSliderPosition(scrollbar->maximum());
    }
}

void QTerminal::on_cancelButton_clicked()
{
        char x = '\003';
    QByteArray stop_signal = &x;
    ROSProcess->write(stop_signal,1);
}
