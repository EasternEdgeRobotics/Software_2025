/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *action;
    QAction *actionOpen;
    QWidget *centralwidget;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QLabel *label;
    QLabel *label_2;
    QMenuBar *menubar;
    QMenu *menuOpen_File;
    QMenu *menuSelect_Pilot;
    QMenu *menuSettings;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(775, 598);
        action = new QAction(MainWindow);
        action->setObjectName("action");
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName("actionOpen");
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName("pushButton");
        pushButton->setGeometry(QRect(20, 330, 161, 31));
        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName("pushButton_2");
        pushButton_2->setGeometry(QRect(20, 370, 161, 31));
        label = new QLabel(centralwidget);
        label->setObjectName("label");
        label->setGeometry(QRect(210, 0, 371, 221));
        label->setPixmap(QPixmap(QString::fromUtf8(":/eer-yellow.xSeVkOog.svg")));
        label->setScaledContents(true);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(310, 300, 431, 141));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 775, 27));
        menuOpen_File = new QMenu(menubar);
        menuOpen_File->setObjectName("menuOpen_File");
        menuSelect_Pilot = new QMenu(menubar);
        menuSelect_Pilot->setObjectName("menuSelect_Pilot");
        menuSettings = new QMenu(menubar);
        menuSettings->setObjectName("menuSettings");
        MainWindow->setMenuBar(menubar);

        menubar->addAction(menuOpen_File->menuAction());
        menubar->addAction(menuSelect_Pilot->menuAction());
        menubar->addAction(menuSettings->menuAction());
        menuOpen_File->addAction(actionOpen);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Eastern Edge Robots Software Tool Kit", nullptr));
        action->setText(QCoreApplication::translate("MainWindow", "EER Tool", nullptr));
        actionOpen->setText(QCoreApplication::translate("MainWindow", "Open", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "Launch Frontend", nullptr));
        pushButton_2->setText(QCoreApplication::translate("MainWindow", "Launch Backend", nullptr));
        label->setText(QString());
        label_2->setText(QString());
        menuOpen_File->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuSelect_Pilot->setTitle(QCoreApplication::translate("MainWindow", "Select Pilot", nullptr));
        menuSettings->setTitle(QCoreApplication::translate("MainWindow", "Settings", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
