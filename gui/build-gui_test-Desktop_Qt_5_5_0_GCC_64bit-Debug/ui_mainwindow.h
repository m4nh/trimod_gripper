/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QVBoxLayout *joints_group;
    QHBoxLayout *horizontalLayout;
    QLabel *j0_label;
    QSlider *j0_slider;
    QLCDNumber *j0_lcd;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label;
    QSlider *j1_slider;
    QLCDNumber *j1_lcd;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_2;
    QSlider *j2_slider;
    QLCDNumber *j2_lcd;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QSlider *j3_slider;
    QLCDNumber *j3_lcd;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QSlider *j4_slider;
    QLCDNumber *j4_lcd;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_5;
    QSlider *j5_slider;
    QLCDNumber *j5_lcd;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_6;
    QSlider *j6_slider;
    QLCDNumber *j6_lcd;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(416, 384);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        joints_group = new QVBoxLayout();
        joints_group->setSpacing(6);
        joints_group->setObjectName(QStringLiteral("joints_group"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        j0_label = new QLabel(centralWidget);
        j0_label->setObjectName(QStringLiteral("j0_label"));

        horizontalLayout->addWidget(j0_label);

        j0_slider = new QSlider(centralWidget);
        j0_slider->setObjectName(QStringLiteral("j0_slider"));
        j0_slider->setMinimum(-180);
        j0_slider->setMaximum(180);
        j0_slider->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(j0_slider);

        j0_lcd = new QLCDNumber(centralWidget);
        j0_lcd->setObjectName(QStringLiteral("j0_lcd"));

        horizontalLayout->addWidget(j0_lcd);


        joints_group->addLayout(horizontalLayout);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout_8->addWidget(label);

        j1_slider = new QSlider(centralWidget);
        j1_slider->setObjectName(QStringLiteral("j1_slider"));
        j1_slider->setMinimum(-180);
        j1_slider->setMaximum(180);
        j1_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_8->addWidget(j1_slider);

        j1_lcd = new QLCDNumber(centralWidget);
        j1_lcd->setObjectName(QStringLiteral("j1_lcd"));

        horizontalLayout_8->addWidget(j1_lcd);


        joints_group->addLayout(horizontalLayout_8);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_5->addWidget(label_2);

        j2_slider = new QSlider(centralWidget);
        j2_slider->setObjectName(QStringLiteral("j2_slider"));
        j2_slider->setMinimum(-180);
        j2_slider->setMaximum(180);
        j2_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(j2_slider);

        j2_lcd = new QLCDNumber(centralWidget);
        j2_lcd->setObjectName(QStringLiteral("j2_lcd"));

        horizontalLayout_5->addWidget(j2_lcd);


        joints_group->addLayout(horizontalLayout_5);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_3->addWidget(label_3);

        j3_slider = new QSlider(centralWidget);
        j3_slider->setObjectName(QStringLiteral("j3_slider"));
        j3_slider->setMinimum(-180);
        j3_slider->setMaximum(180);
        j3_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(j3_slider);

        j3_lcd = new QLCDNumber(centralWidget);
        j3_lcd->setObjectName(QStringLiteral("j3_lcd"));

        horizontalLayout_3->addWidget(j3_lcd);


        joints_group->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_4->addWidget(label_4);

        j4_slider = new QSlider(centralWidget);
        j4_slider->setObjectName(QStringLiteral("j4_slider"));
        j4_slider->setMinimum(-180);
        j4_slider->setMaximum(180);
        j4_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(j4_slider);

        j4_lcd = new QLCDNumber(centralWidget);
        j4_lcd->setObjectName(QStringLiteral("j4_lcd"));

        horizontalLayout_4->addWidget(j4_lcd);


        joints_group->addLayout(horizontalLayout_4);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        horizontalLayout_7->addWidget(label_5);

        j5_slider = new QSlider(centralWidget);
        j5_slider->setObjectName(QStringLiteral("j5_slider"));
        j5_slider->setMinimum(-180);
        j5_slider->setMaximum(180);
        j5_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_7->addWidget(j5_slider);

        j5_lcd = new QLCDNumber(centralWidget);
        j5_lcd->setObjectName(QStringLiteral("j5_lcd"));

        horizontalLayout_7->addWidget(j5_lcd);


        joints_group->addLayout(horizontalLayout_7);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_2->addWidget(label_6);

        j6_slider = new QSlider(centralWidget);
        j6_slider->setObjectName(QStringLiteral("j6_slider"));
        j6_slider->setMinimum(-180);
        j6_slider->setMaximum(180);
        j6_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(j6_slider);

        j6_lcd = new QLCDNumber(centralWidget);
        j6_lcd->setObjectName(QStringLiteral("j6_lcd"));

        horizontalLayout_2->addWidget(j6_lcd);


        joints_group->addLayout(horizontalLayout_2);


        gridLayout->addLayout(joints_group, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        j0_label->setText(QApplication::translate("MainWindow", "J0", 0));
        label->setText(QApplication::translate("MainWindow", "J1", 0));
        label_2->setText(QApplication::translate("MainWindow", "J2", 0));
        label_3->setText(QApplication::translate("MainWindow", "J3", 0));
        label_4->setText(QApplication::translate("MainWindow", "J4", 0));
        label_5->setText(QApplication::translate("MainWindow", "J5", 0));
        label_6->setText(QApplication::translate("MainWindow", "J6", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
