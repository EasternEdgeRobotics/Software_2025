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
    QProcess process;
    process.start("/bin/bash", QStringList()<<"helloworld.sh");
    process.waitForFinished();

    QTextStream out(stdout);
    QString output = process.readAllStandardOutput();

    ui->label_2->setText(output);
}


void MainWindow::on_pushButton_2_released()
{

    QProcess process;
    process.start("/bin/bash", QStringList()<<"button2.sh");
    process.waitForFinished();

    QTextStream out(stdout);
    QString output = process.readAllStandardOutput();

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


