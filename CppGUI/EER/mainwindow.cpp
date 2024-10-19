#include "mainwindow.h"
#include "qstatusbar.h"
#include "ui_mainwindow.h"
#include "QProcess"
#include "QTextStream"
#include "QFileDialog"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_released()
{

    QString command = R"(
        cmds=("echo 'hello'");
        for i in "${cmds[@]}"; do
            gnome-terminal -- sh -c "$i; exec /bin/bash" &
        done;
        echo "I just opened ${#cmds[@]} terminals!";
    )";

    QStringList arg;
    arg << "-c" << command;

    QProcess process;
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start("/bin/bash", arg);

    if (!process.waitForStarted()) {
        qDebug() << "Error starting process:" << process.errorString();
        return;
    }

    if (!process.waitForFinished()) {
        qDebug() << "Error: Process did not finish:" << process.errorString();
        return;
    }

    QString output = process.readAllStandardOutput().trimmed();
    qDebug() << "Output:" << output;

    if (output.isEmpty()) {
        qDebug() << "No output received.";
    }

    ui->label_2->setText(output);
}


void MainWindow::on_pushButton_2_released()
{
    QString command = R"(
        echo 'Hello, this is button 2 and should only appear in the text screen';
    )";

    QStringList arg;
    arg << "-c" << command;

    QProcess process;
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start("/bin/bash", arg);

    if (!process.waitForStarted()) {
        qDebug() << "Error starting process:" << process.errorString();
        return;
    }

    if (!process.waitForFinished()) {
        qDebug() << "Error: Process did not finish:" << process.errorString();
        return;
    }

    QString output = process.readAllStandardOutput().trimmed();
    qDebug() << "Output:" << output;

    if (output.isEmpty()) {
        qDebug() << "No output received.";
    }

    ui->label_2->setText(output);
}


void MainWindow::on_actionOpen_triggered()
{
    QFileDialog fileDialog(this, tr("Open Document"), QDir::currentPath());
    while (fileDialog.exec() == QDialog::Accepted
           && !openFile(fileDialog.selectedFiles().constFirst())) {
    }
}

bool MainWindow::openFile(const QString &fileName)
{
    QFile *file = new QFile(fileName);
    if (!file->exists()) {
        statusBar()->showMessage(tr("File %1 could not be opened")
                                     .arg(QDir::toNativeSeparators(fileName)));
        delete file;
        return false;
    }

    return true;
}


