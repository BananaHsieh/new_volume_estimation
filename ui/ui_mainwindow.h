/********************************************************************************
** Form generated from reading UI file 'ui_mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout;
    QStackedWidget *stackedWidget;
    QWidget *homePage;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_8;
    QPushButton *buttonToROIPage;
    QPushButton *buttonToPlanePage;
    QSpacerItem *verticalSpacer_2;
    QWidget *setROIPage;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_7;
    QLineEdit *lineEditMovestep;
    QPushButton *buttonLoadLocalROI;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *maxZ;
    QLabel *label_2;
    QPushButton *buttonIncreaseMaxZ;
    QPushButton *buttonDecreaseMaxZ;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *xx;
    QLabel *label_3;
    QHBoxLayout *maxX;
    QPushButton *buttonDecreaseMaxX;
    QPushButton *buttonIncreaseMaxX;
    QLabel *label_4;
    QHBoxLayout *minX;
    QPushButton *buttonDecreaseMinX;
    QPushButton *buttonIncreaseMinX;
    QVBoxLayout *yy;
    QLabel *label_6;
    QHBoxLayout *maxY;
    QPushButton *buttonDecreaseMaxY;
    QPushButton *buttonIncreaseMaxY;
    QLabel *label_5;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *buttonDecreaseMinY;
    QPushButton *buttonIncreaseMinY;
    QVBoxLayout *minZ;
    QLabel *label;
    QPushButton *buttonIncreaseMinZ;
    QPushButton *buttonDecreaseMinZ;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *buttonBackToHome;
    QSpacerItem *horizontalSpacer;
    QPushButton *buttonSaveLocalROI;
    QWidget *selectPlanePage;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_3;
    QSlider *verticalSlider;
    QVTKWidget *qvtkWidget_plane;
    QPushButton *buttonApplyPlane;
    QSpacerItem *verticalSpacer;
    QPushButton *buttonBackToHome2;
    QHBoxLayout *horizontalLayout;
    QVTKWidget *qvtkWidget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(894, 666);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout_2 = new QHBoxLayout(centralwidget);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        stackedWidget = new QStackedWidget(centralwidget);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        homePage = new QWidget();
        homePage->setObjectName(QString::fromUtf8("homePage"));
        verticalLayout_2 = new QVBoxLayout(homePage);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        label_8 = new QLabel(homePage);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_6->addWidget(label_8, 0, Qt::AlignHCenter);

        buttonToROIPage = new QPushButton(homePage);
        buttonToROIPage->setObjectName(QString::fromUtf8("buttonToROIPage"));

        verticalLayout_6->addWidget(buttonToROIPage);

        buttonToPlanePage = new QPushButton(homePage);
        buttonToPlanePage->setObjectName(QString::fromUtf8("buttonToPlanePage"));

        verticalLayout_6->addWidget(buttonToPlanePage);


        verticalLayout_2->addLayout(verticalLayout_6);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        stackedWidget->addWidget(homePage);
        setROIPage = new QWidget();
        setROIPage->setObjectName(QString::fromUtf8("setROIPage"));
        verticalLayout_3 = new QVBoxLayout(setROIPage);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_7 = new QLabel(setROIPage);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_5->addWidget(label_7, 0, Qt::AlignLeft);

        lineEditMovestep = new QLineEdit(setROIPage);
        lineEditMovestep->setObjectName(QString::fromUtf8("lineEditMovestep"));

        horizontalLayout_5->addWidget(lineEditMovestep, 0, Qt::AlignLeft);

        buttonLoadLocalROI = new QPushButton(setROIPage);
        buttonLoadLocalROI->setObjectName(QString::fromUtf8("buttonLoadLocalROI"));

        horizontalLayout_5->addWidget(buttonLoadLocalROI);

        horizontalLayout_5->setStretch(0, 1);
        horizontalLayout_5->setStretch(1, 3);

        verticalLayout_3->addLayout(horizontalLayout_5);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        maxZ = new QVBoxLayout();
        maxZ->setSpacing(0);
        maxZ->setObjectName(QString::fromUtf8("maxZ"));
        label_2 = new QLabel(setROIPage);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        maxZ->addWidget(label_2, 0, Qt::AlignHCenter);

        buttonIncreaseMaxZ = new QPushButton(setROIPage);
        buttonIncreaseMaxZ->setObjectName(QString::fromUtf8("buttonIncreaseMaxZ"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(buttonIncreaseMaxZ->sizePolicy().hasHeightForWidth());
        buttonIncreaseMaxZ->setSizePolicy(sizePolicy);

        maxZ->addWidget(buttonIncreaseMaxZ);

        buttonDecreaseMaxZ = new QPushButton(setROIPage);
        buttonDecreaseMaxZ->setObjectName(QString::fromUtf8("buttonDecreaseMaxZ"));
        sizePolicy.setHeightForWidth(buttonDecreaseMaxZ->sizePolicy().hasHeightForWidth());
        buttonDecreaseMaxZ->setSizePolicy(sizePolicy);

        maxZ->addWidget(buttonDecreaseMaxZ);


        verticalLayout_4->addLayout(maxZ);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        xx = new QVBoxLayout();
        xx->setObjectName(QString::fromUtf8("xx"));
        label_3 = new QLabel(setROIPage);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        xx->addWidget(label_3, 0, Qt::AlignHCenter);

        maxX = new QHBoxLayout();
        maxX->setObjectName(QString::fromUtf8("maxX"));
        buttonDecreaseMaxX = new QPushButton(setROIPage);
        buttonDecreaseMaxX->setObjectName(QString::fromUtf8("buttonDecreaseMaxX"));
        sizePolicy.setHeightForWidth(buttonDecreaseMaxX->sizePolicy().hasHeightForWidth());
        buttonDecreaseMaxX->setSizePolicy(sizePolicy);

        maxX->addWidget(buttonDecreaseMaxX);

        buttonIncreaseMaxX = new QPushButton(setROIPage);
        buttonIncreaseMaxX->setObjectName(QString::fromUtf8("buttonIncreaseMaxX"));
        sizePolicy.setHeightForWidth(buttonIncreaseMaxX->sizePolicy().hasHeightForWidth());
        buttonIncreaseMaxX->setSizePolicy(sizePolicy);

        maxX->addWidget(buttonIncreaseMaxX);


        xx->addLayout(maxX);

        label_4 = new QLabel(setROIPage);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        xx->addWidget(label_4, 0, Qt::AlignHCenter);

        minX = new QHBoxLayout();
        minX->setObjectName(QString::fromUtf8("minX"));
        buttonDecreaseMinX = new QPushButton(setROIPage);
        buttonDecreaseMinX->setObjectName(QString::fromUtf8("buttonDecreaseMinX"));
        sizePolicy.setHeightForWidth(buttonDecreaseMinX->sizePolicy().hasHeightForWidth());
        buttonDecreaseMinX->setSizePolicy(sizePolicy);

        minX->addWidget(buttonDecreaseMinX);

        buttonIncreaseMinX = new QPushButton(setROIPage);
        buttonIncreaseMinX->setObjectName(QString::fromUtf8("buttonIncreaseMinX"));
        sizePolicy.setHeightForWidth(buttonIncreaseMinX->sizePolicy().hasHeightForWidth());
        buttonIncreaseMinX->setSizePolicy(sizePolicy);

        minX->addWidget(buttonIncreaseMinX);


        xx->addLayout(minX);

        xx->setStretch(0, 1);
        xx->setStretch(1, 8);
        xx->setStretch(2, 1);
        xx->setStretch(3, 8);

        horizontalLayout_4->addLayout(xx);

        yy = new QVBoxLayout();
        yy->setObjectName(QString::fromUtf8("yy"));
        label_6 = new QLabel(setROIPage);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        yy->addWidget(label_6, 0, Qt::AlignHCenter);

        maxY = new QHBoxLayout();
        maxY->setObjectName(QString::fromUtf8("maxY"));
        buttonDecreaseMaxY = new QPushButton(setROIPage);
        buttonDecreaseMaxY->setObjectName(QString::fromUtf8("buttonDecreaseMaxY"));
        sizePolicy.setHeightForWidth(buttonDecreaseMaxY->sizePolicy().hasHeightForWidth());
        buttonDecreaseMaxY->setSizePolicy(sizePolicy);

        maxY->addWidget(buttonDecreaseMaxY);

        buttonIncreaseMaxY = new QPushButton(setROIPage);
        buttonIncreaseMaxY->setObjectName(QString::fromUtf8("buttonIncreaseMaxY"));
        sizePolicy.setHeightForWidth(buttonIncreaseMaxY->sizePolicy().hasHeightForWidth());
        buttonIncreaseMaxY->setSizePolicy(sizePolicy);

        maxY->addWidget(buttonIncreaseMaxY);


        yy->addLayout(maxY);

        label_5 = new QLabel(setROIPage);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        yy->addWidget(label_5, 0, Qt::AlignHCenter);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        buttonDecreaseMinY = new QPushButton(setROIPage);
        buttonDecreaseMinY->setObjectName(QString::fromUtf8("buttonDecreaseMinY"));
        sizePolicy.setHeightForWidth(buttonDecreaseMinY->sizePolicy().hasHeightForWidth());
        buttonDecreaseMinY->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(buttonDecreaseMinY);

        buttonIncreaseMinY = new QPushButton(setROIPage);
        buttonIncreaseMinY->setObjectName(QString::fromUtf8("buttonIncreaseMinY"));
        sizePolicy.setHeightForWidth(buttonIncreaseMinY->sizePolicy().hasHeightForWidth());
        buttonIncreaseMinY->setSizePolicy(sizePolicy);

        horizontalLayout_6->addWidget(buttonIncreaseMinY);


        yy->addLayout(horizontalLayout_6);

        yy->setStretch(0, 1);
        yy->setStretch(1, 8);
        yy->setStretch(2, 1);
        yy->setStretch(3, 8);

        horizontalLayout_4->addLayout(yy);


        verticalLayout_4->addLayout(horizontalLayout_4);

        minZ = new QVBoxLayout();
        minZ->setSpacing(0);
        minZ->setObjectName(QString::fromUtf8("minZ"));
        label = new QLabel(setROIPage);
        label->setObjectName(QString::fromUtf8("label"));

        minZ->addWidget(label, 0, Qt::AlignHCenter);

        buttonIncreaseMinZ = new QPushButton(setROIPage);
        buttonIncreaseMinZ->setObjectName(QString::fromUtf8("buttonIncreaseMinZ"));
        sizePolicy.setHeightForWidth(buttonIncreaseMinZ->sizePolicy().hasHeightForWidth());
        buttonIncreaseMinZ->setSizePolicy(sizePolicy);

        minZ->addWidget(buttonIncreaseMinZ);

        buttonDecreaseMinZ = new QPushButton(setROIPage);
        buttonDecreaseMinZ->setObjectName(QString::fromUtf8("buttonDecreaseMinZ"));
        sizePolicy.setHeightForWidth(buttonDecreaseMinZ->sizePolicy().hasHeightForWidth());
        buttonDecreaseMinZ->setSizePolicy(sizePolicy);

        minZ->addWidget(buttonDecreaseMinZ);


        verticalLayout_4->addLayout(minZ);

        verticalLayout_4->setStretch(0, 1);
        verticalLayout_4->setStretch(1, 5);
        verticalLayout_4->setStretch(2, 1);

        verticalLayout_3->addLayout(verticalLayout_4);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        buttonBackToHome = new QPushButton(setROIPage);
        buttonBackToHome->setObjectName(QString::fromUtf8("buttonBackToHome"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(buttonBackToHome->sizePolicy().hasHeightForWidth());
        buttonBackToHome->setSizePolicy(sizePolicy1);

        horizontalLayout_9->addWidget(buttonBackToHome);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer);

        buttonSaveLocalROI = new QPushButton(setROIPage);
        buttonSaveLocalROI->setObjectName(QString::fromUtf8("buttonSaveLocalROI"));

        horizontalLayout_9->addWidget(buttonSaveLocalROI);


        verticalLayout_3->addLayout(horizontalLayout_9);

        stackedWidget->addWidget(setROIPage);
        selectPlanePage = new QWidget();
        selectPlanePage->setObjectName(QString::fromUtf8("selectPlanePage"));
        verticalLayout_5 = new QVBoxLayout(selectPlanePage);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalSlider = new QSlider(selectPlanePage);
        verticalSlider->setObjectName(QString::fromUtf8("verticalSlider"));
        verticalSlider->setOrientation(Qt::Vertical);

        horizontalLayout_3->addWidget(verticalSlider);

        qvtkWidget_plane = new QVTKWidget(selectPlanePage);
        qvtkWidget_plane->setObjectName(QString::fromUtf8("qvtkWidget_plane"));

        horizontalLayout_3->addWidget(qvtkWidget_plane);


        verticalLayout_5->addLayout(horizontalLayout_3);

        buttonApplyPlane = new QPushButton(selectPlanePage);
        buttonApplyPlane->setObjectName(QString::fromUtf8("buttonApplyPlane"));

        verticalLayout_5->addWidget(buttonApplyPlane);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer);

        buttonBackToHome2 = new QPushButton(selectPlanePage);
        buttonBackToHome2->setObjectName(QString::fromUtf8("buttonBackToHome2"));

        verticalLayout_5->addWidget(buttonBackToHome2, 0, Qt::AlignLeft);

        stackedWidget->addWidget(selectPlanePage);

        verticalLayout->addWidget(stackedWidget);


        horizontalLayout_2->addLayout(verticalLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));

        horizontalLayout->addWidget(qvtkWidget);


        horizontalLayout_2->addLayout(horizontalLayout);

        horizontalLayout_2->setStretch(0, 2);
        horizontalLayout_2->setStretch(1, 3);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 894, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        stackedWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "SETTING", nullptr));
        buttonToROIPage->setText(QCoreApplication::translate("MainWindow", "Select ROI", nullptr));
        buttonToPlanePage->setText(QCoreApplication::translate("MainWindow", "Select Plane", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "move step:", nullptr));
        buttonLoadLocalROI->setText(QCoreApplication::translate("MainWindow", "LoadLocalROI", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Top", nullptr));
        buttonIncreaseMaxZ->setText(QCoreApplication::translate("MainWindow", "UP", nullptr));
        buttonDecreaseMaxZ->setText(QCoreApplication::translate("MainWindow", "DOWN", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        buttonDecreaseMaxX->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        buttonIncreaseMaxX->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        buttonDecreaseMinX->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        buttonIncreaseMinX->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Front", nullptr));
        buttonDecreaseMaxY->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        buttonIncreaseMaxY->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Back", nullptr));
        buttonDecreaseMinY->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        buttonIncreaseMinY->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Bottom", nullptr));
        buttonIncreaseMinZ->setText(QCoreApplication::translate("MainWindow", "UP", nullptr));
        buttonDecreaseMinZ->setText(QCoreApplication::translate("MainWindow", "DOWN", nullptr));
        buttonBackToHome->setText(QCoreApplication::translate("MainWindow", "back", nullptr));
        buttonSaveLocalROI->setText(QCoreApplication::translate("MainWindow", "SaveLocalROI", nullptr));
        buttonApplyPlane->setText(QCoreApplication::translate("MainWindow", "Apply", nullptr));
        buttonBackToHome2->setText(QCoreApplication::translate("MainWindow", "back", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
