/********************************************************************************
** Form generated from reading UI file 'controlform.ui'
**
** Created: Thu Aug 8 11:05:00 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROLFORM_H
#define UI_CONTROLFORM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ControlForm
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_2;
    QPushButton *topButton;
    QPushButton *mdlButton;
    QPushButton *btmButton;
    QPushButton *stpButton;
    QVBoxLayout *verticalLayout;
    QPushButton *posButton;
    QPushButton *velButton;
    QPushButton *hndButton;
    QPushButton *calButton;
    QVBoxLayout *verticalLayout_5;
    QSlider *F1Slider;
    QLabel *label_4;
    QVBoxLayout *verticalLayout_6;
    QSlider *F2Slider;
    QLabel *label_5;
    QVBoxLayout *verticalLayout_4;
    QSlider *F3Slider;
    QLabel *label_3;
    QVBoxLayout *verticalLayout_3;
    QSlider *F4Slider;
    QLabel *label_2;
    QVBoxLayout *verticalLayout_7;
    QSlider *F5Slider;
    QLabel *label_6;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ControlForm)
    {
        if (ControlForm->objectName().isEmpty())
            ControlForm->setObjectName(QString::fromUtf8("ControlForm"));
        ControlForm->resize(398, 193);
        centralwidget = new QWidget(ControlForm);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        topButton = new QPushButton(centralwidget);
        topButton->setObjectName(QString::fromUtf8("topButton"));
        topButton->setMaximumSize(QSize(100, 16777215));

        verticalLayout_2->addWidget(topButton);

        mdlButton = new QPushButton(centralwidget);
        mdlButton->setObjectName(QString::fromUtf8("mdlButton"));
        mdlButton->setMaximumSize(QSize(100, 16777215));

        verticalLayout_2->addWidget(mdlButton);

        btmButton = new QPushButton(centralwidget);
        btmButton->setObjectName(QString::fromUtf8("btmButton"));
        btmButton->setMaximumSize(QSize(100, 16777215));

        verticalLayout_2->addWidget(btmButton);

        stpButton = new QPushButton(centralwidget);
        stpButton->setObjectName(QString::fromUtf8("stpButton"));
        stpButton->setMaximumSize(QSize(100, 16777215));

        verticalLayout_2->addWidget(stpButton);


        gridLayout->addLayout(verticalLayout_2, 0, 0, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        posButton = new QPushButton(centralwidget);
        posButton->setObjectName(QString::fromUtf8("posButton"));
        posButton->setMaximumSize(QSize(100, 16777215));
        posButton->setCheckable(true);

        verticalLayout->addWidget(posButton);

        velButton = new QPushButton(centralwidget);
        velButton->setObjectName(QString::fromUtf8("velButton"));
        velButton->setMaximumSize(QSize(100, 16777215));
        velButton->setCheckable(true);

        verticalLayout->addWidget(velButton);

        hndButton = new QPushButton(centralwidget);
        hndButton->setObjectName(QString::fromUtf8("hndButton"));
        hndButton->setMaximumSize(QSize(100, 16777215));
        hndButton->setCheckable(false);

        verticalLayout->addWidget(hndButton);

        calButton = new QPushButton(centralwidget);
        calButton->setObjectName(QString::fromUtf8("calButton"));
        calButton->setMaximumSize(QSize(100, 16777215));
        calButton->setCheckable(false);

        verticalLayout->addWidget(calButton);


        gridLayout->addLayout(verticalLayout, 0, 1, 1, 1);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        F1Slider = new QSlider(centralwidget);
        F1Slider->setObjectName(QString::fromUtf8("F1Slider"));
        F1Slider->setMinimum(0);
        F1Slider->setMaximum(100);
        F1Slider->setValue(0);
        F1Slider->setOrientation(Qt::Vertical);

        verticalLayout_5->addWidget(F1Slider);

        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setMaximumSize(QSize(25, 16777215));
        label_4->setAlignment(Qt::AlignCenter);

        verticalLayout_5->addWidget(label_4);


        gridLayout->addLayout(verticalLayout_5, 0, 2, 1, 1);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        F2Slider = new QSlider(centralwidget);
        F2Slider->setObjectName(QString::fromUtf8("F2Slider"));
        F2Slider->setMinimum(0);
        F2Slider->setMaximum(100);
        F2Slider->setValue(0);
        F2Slider->setOrientation(Qt::Vertical);

        verticalLayout_6->addWidget(F2Slider);

        label_5 = new QLabel(centralwidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setMaximumSize(QSize(25, 16777215));
        label_5->setAlignment(Qt::AlignCenter);

        verticalLayout_6->addWidget(label_5);


        gridLayout->addLayout(verticalLayout_6, 0, 3, 1, 1);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        F3Slider = new QSlider(centralwidget);
        F3Slider->setObjectName(QString::fromUtf8("F3Slider"));
        F3Slider->setMinimum(0);
        F3Slider->setMaximum(100);
        F3Slider->setValue(0);
        F3Slider->setOrientation(Qt::Vertical);

        verticalLayout_4->addWidget(F3Slider);

        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setMaximumSize(QSize(25, 16777215));
        label_3->setAlignment(Qt::AlignCenter);

        verticalLayout_4->addWidget(label_3);


        gridLayout->addLayout(verticalLayout_4, 0, 4, 1, 1);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        F4Slider = new QSlider(centralwidget);
        F4Slider->setObjectName(QString::fromUtf8("F4Slider"));
        F4Slider->setMinimum(0);
        F4Slider->setMaximum(100);
        F4Slider->setValue(0);
        F4Slider->setOrientation(Qt::Vertical);

        verticalLayout_3->addWidget(F4Slider);

        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setMaximumSize(QSize(25, 16777215));
        label_2->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(label_2);


        gridLayout->addLayout(verticalLayout_3, 0, 5, 1, 1);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        F5Slider = new QSlider(centralwidget);
        F5Slider->setObjectName(QString::fromUtf8("F5Slider"));
        F5Slider->setMinimum(0);
        F5Slider->setMaximum(100);
        F5Slider->setValue(0);
        F5Slider->setOrientation(Qt::Vertical);

        verticalLayout_7->addWidget(F5Slider);

        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setMaximumSize(QSize(25, 16777215));
        label_6->setAlignment(Qt::AlignCenter);

        verticalLayout_7->addWidget(label_6);


        gridLayout->addLayout(verticalLayout_7, 0, 6, 1, 1);

        ControlForm->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ControlForm);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 398, 25));
        ControlForm->setMenuBar(menubar);
        statusbar = new QStatusBar(ControlForm);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        ControlForm->setStatusBar(statusbar);

        retranslateUi(ControlForm);

        QMetaObject::connectSlotsByName(ControlForm);
    } // setupUi

    void retranslateUi(QMainWindow *ControlForm)
    {
        ControlForm->setWindowTitle(QApplication::translate("ControlForm", "HANDLE control", 0, QApplication::UnicodeUTF8));
        topButton->setText(QApplication::translate("ControlForm", "Top", 0, QApplication::UnicodeUTF8));
        mdlButton->setText(QApplication::translate("ControlForm", "Middle", 0, QApplication::UnicodeUTF8));
        btmButton->setText(QApplication::translate("ControlForm", "Bottom", 0, QApplication::UnicodeUTF8));
        stpButton->setText(QApplication::translate("ControlForm", "STOP", 0, QApplication::UnicodeUTF8));
        posButton->setText(QApplication::translate("ControlForm", "Position", 0, QApplication::UnicodeUTF8));
        velButton->setText(QApplication::translate("ControlForm", "Velocity", 0, QApplication::UnicodeUTF8));
        hndButton->setText(QApplication::translate("ControlForm", "Switch to Left", 0, QApplication::UnicodeUTF8));
        calButton->setText(QApplication::translate("ControlForm", "Calibrate", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("ControlForm", "F1", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("ControlForm", "F2", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("ControlForm", "F3", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("ControlForm", "F3A", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("ControlForm", "S", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ControlForm: public Ui_ControlForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROLFORM_H
