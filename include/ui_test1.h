/********************************************************************************
** Form generated from reading UI file 'test1gvMrVN.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef TEST1GVMRVN_H
#define TEST1GVMRVN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QString darkStyleSheet = R"(
    QWidget {
        background-color: #2E2E2E;
        color: #FFFFFF;
    }
    QMenuBar {
        background-color: #2E2E2E;
        color: #FFFFFF;
    }
    QMenuBar::item {
        background-color: #2E2E2E;
        color: #FFFFFF;
    }
    QMenuBar::item::selected {
        background-color: #555555;
    }
    QMenu {
        background-color: #2E2E2E;
        color: #FFFFFF;
    }
    QMenu::item::selected {
        background-color: #555555;
    }
    QPushButton {
        background-color: #4A4A4A;
        color: #FFFFFF;
        border: none;
        padding: 5px;
    }
    QPushButton::hover {
        background-color: #555555;
    }
    QPushButton::pressed {
        background-color: #666666;
    }
    QTextBrowser {
        background-color: #2E2E2E;
        color: #FFFFFF;
        border: none;
    }
    QMainWindow {
        background-color: #2E2E2E;
    }
    ColorButton {
        background-color: #4A4A4A;
        color: #000000;
        border: none;
        padding: 5px;
    }
    ColorButton:hover {
        background-color: #555555;
    }
    ColorButton:pressed {
        background-color: #666666;
    }
)";

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *tabWidget;
    QWidget *tab;
    QLabel *label_4;
    QFrame *frame_2;
    QWidget *widget;
    QSlider *verticalSlider_2;
    QSlider *verticalSlider;
    QSlider *horizontalSlider;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QFrame *frame;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QToolButton *toolButton;
    QFrame *frame_4;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_6;
    QLabel *label_5;
    QLabel *label_7;
    QLabel *label_13;
    QVBoxLayout *verticalLayout_3;
    QTextBrowser *textBrowser_2;
    QTextBrowser *textBrowser_3;
    QTextBrowser *textBrowser;
    QTextBrowser *textBrowser_16;
    QWidget *tab_2;
    QLabel *label_12;
    QFrame *frame_3;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_12;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QVBoxLayout *verticalLayout_9;
    QTextBrowser *textBrowser_4;
    QTextBrowser *textBrowser_7;
    QTextBrowser *textBrowser_10;
    QTextBrowser *textBrowser_13;
    QVBoxLayout *verticalLayout_10;
    QTextBrowser *textBrowser_5;
    QTextBrowser *textBrowser_8;
    QTextBrowser *textBrowser_11;
    QTextBrowser *textBrowser_14;
    QVBoxLayout *verticalLayout_6;
    QTextBrowser *textBrowser_6;
    QTextBrowser *textBrowser_9;
    QTextBrowser *textBrowser_12;
    QTextBrowser *textBrowser_15;
    QWidget *tab_3;
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QWidget *widget_2;
    QFrame *frame_5;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_5;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox;
    QCheckBox *checkBox_3;
    QCheckBox *checkBox_4;
    QCheckBox *checkBox_5;
    QCheckBox *checkBox_6;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(906, 547);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(10, 10, 871, 531));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        label_4 = new QLabel(tab);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(180, 420, 421, 51));
        QFont font;
        font.setPointSize(21);
        label_4->setFont(font);
        frame_2 = new QFrame(tab);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setGeometry(QRect(10, 30, 491, 371));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        widget = new QWidget(frame_2);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(10, 10, 471, 351));
        verticalSlider_2 = new QSlider(widget);
        verticalSlider_2->setObjectName(QString::fromUtf8("verticalSlider_2"));
        verticalSlider_2->setGeometry(QRect(230, 10, 31, 281));
        verticalSlider_2->setAutoFillBackground(false);
        verticalSlider_2->setMinimum(-50);
        verticalSlider_2->setMaximum(50);
        verticalSlider_2->setSliderPosition(0);
        verticalSlider_2->setTracking(true);
        verticalSlider_2->setOrientation(Qt::Vertical);
        verticalSlider_2->setInvertedAppearance(false);
        verticalSlider_2->setTickPosition(QSlider::NoTicks);
        verticalSlider = new QSlider(widget);
        verticalSlider->setObjectName(QString::fromUtf8("verticalSlider"));
        verticalSlider->setGeometry(QRect(390, 10, 31, 291));
        verticalSlider->setOrientation(Qt::Vertical);
        verticalSlider->setTickPosition(QSlider::NoTicks);
        horizontalSlider = new QSlider(widget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setGeometry(QRect(30, 200, 161, 18));
        horizontalSlider->setLayoutDirection(Qt::LeftToRight);
        horizontalSlider->setAutoFillBackground(true);
        horizontalSlider->setOrientation(Qt::Horizontal);
        layoutWidget = new QWidget(widget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(30, 320, 451, 23));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setLayoutDirection(Qt::LeftToRight);

        horizontalLayout->addWidget(label);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout->addWidget(label_3);

        frame = new QFrame(tab);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(710, 10, 141, 131));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout = new QVBoxLayout(frame);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton = new QPushButton(frame);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setLayoutDirection(Qt::RightToLeft);
        pushButton->setAutoDefault(false);
        pushButton->setFlat(false);

        verticalLayout->addWidget(pushButton);

        pushButton_2 = new QPushButton(frame);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setCursor(QCursor(Qt::ArrowCursor));
        pushButton_2->setAcceptDrops(false);

        verticalLayout->addWidget(pushButton_2);

        toolButton = new QToolButton(tab);
        toolButton->setObjectName(QString::fromUtf8("toolButton"));
        toolButton->setGeometry(QRect(830, 450, 30, 28));
        frame_4 = new QFrame(tab);
        frame_4->setObjectName(QString::fromUtf8("frame_4"));
        frame_4->setGeometry(QRect(540, 230, 311, 171));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);
        horizontalLayoutWidget = new QWidget(frame_4);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 10, 291, 151));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_6 = new QLabel(horizontalLayoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_2->addWidget(label_6);

        label_5 = new QLabel(horizontalLayoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout_2->addWidget(label_5);

        label_7 = new QLabel(horizontalLayoutWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_2->addWidget(label_7);

        label_13 = new QLabel(horizontalLayoutWidget);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        verticalLayout_2->addWidget(label_13);


        horizontalLayout_2->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        textBrowser_2 = new QTextBrowser(horizontalLayoutWidget);
        textBrowser_2->setObjectName(QString::fromUtf8("textBrowser_2"));

        verticalLayout_3->addWidget(textBrowser_2);

        textBrowser_3 = new QTextBrowser(horizontalLayoutWidget);
        textBrowser_3->setObjectName(QString::fromUtf8("textBrowser_3"));

        verticalLayout_3->addWidget(textBrowser_3);

        textBrowser = new QTextBrowser(horizontalLayoutWidget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));

        verticalLayout_3->addWidget(textBrowser);

        textBrowser_16 = new QTextBrowser(horizontalLayoutWidget);
        textBrowser_16->setObjectName(QString::fromUtf8("textBrowser_16"));

        verticalLayout_3->addWidget(textBrowser_16);


        horizontalLayout_2->addLayout(verticalLayout_3);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        label_12 = new QLabel(tab_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(30, 20, 80, 21));
        frame_3 = new QFrame(tab_2);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setGeometry(QRect(30, 50, 661, 171));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        horizontalLayoutWidget_2 = new QWidget(frame_3);
        horizontalLayoutWidget_2->setObjectName(QString::fromUtf8("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 10, 641, 151));
        horizontalLayout_3 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        label_8 = new QLabel(horizontalLayoutWidget_2);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        verticalLayout_12->addWidget(label_8);

        label_9 = new QLabel(horizontalLayoutWidget_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        verticalLayout_12->addWidget(label_9);

        label_10 = new QLabel(horizontalLayoutWidget_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        verticalLayout_12->addWidget(label_10);

        label_11 = new QLabel(horizontalLayoutWidget_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        verticalLayout_12->addWidget(label_11);


        horizontalLayout_3->addLayout(verticalLayout_12);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        textBrowser_4 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_4->setObjectName(QString::fromUtf8("textBrowser_4"));

        verticalLayout_9->addWidget(textBrowser_4);

        textBrowser_7 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_7->setObjectName(QString::fromUtf8("textBrowser_7"));

        verticalLayout_9->addWidget(textBrowser_7);

        textBrowser_10 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_10->setObjectName(QString::fromUtf8("textBrowser_10"));

        verticalLayout_9->addWidget(textBrowser_10);

        textBrowser_13 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_13->setObjectName(QString::fromUtf8("textBrowser_13"));

        verticalLayout_9->addWidget(textBrowser_13);


        horizontalLayout_3->addLayout(verticalLayout_9);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        textBrowser_5 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_5->setObjectName(QString::fromUtf8("textBrowser_5"));

        verticalLayout_10->addWidget(textBrowser_5);

        textBrowser_8 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_8->setObjectName(QString::fromUtf8("textBrowser_8"));

        verticalLayout_10->addWidget(textBrowser_8);

        textBrowser_11 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_11->setObjectName(QString::fromUtf8("textBrowser_11"));

        verticalLayout_10->addWidget(textBrowser_11);

        textBrowser_14 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_14->setObjectName(QString::fromUtf8("textBrowser_14"));

        verticalLayout_10->addWidget(textBrowser_14);


        horizontalLayout_3->addLayout(verticalLayout_10);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        textBrowser_6 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_6->setObjectName(QString::fromUtf8("textBrowser_6"));

        verticalLayout_6->addWidget(textBrowser_6);

        textBrowser_9 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_9->setObjectName(QString::fromUtf8("textBrowser_9"));

        verticalLayout_6->addWidget(textBrowser_9);

        textBrowser_12 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_12->setObjectName(QString::fromUtf8("textBrowser_12"));

        verticalLayout_6->addWidget(textBrowser_12);

        textBrowser_15 = new QTextBrowser(horizontalLayoutWidget_2);
        textBrowser_15->setObjectName(QString::fromUtf8("textBrowser_15"));

        verticalLayout_6->addWidget(textBrowser_15);


        horizontalLayout_3->addLayout(verticalLayout_6);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        groupBox = new QGroupBox(tab_3);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(700, 10, 151, 131));
        verticalLayoutWidget = new QWidget(groupBox);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 40, 131, 80));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        pushButton_3 = new QPushButton(verticalLayoutWidget);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        verticalLayout_4->addWidget(pushButton_3);

        pushButton_4 = new QPushButton(verticalLayoutWidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        verticalLayout_4->addWidget(pushButton_4);

        widget_2 = new QWidget(tab_3);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setGeometry(QRect(10, 10, 671, 441));
        frame_5 = new QFrame(tab_3);
        frame_5->setObjectName(QString::fromUtf8("frame_5"));
        frame_5->setGeometry(QRect(700, 150, 151, 231));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);
        verticalLayoutWidget_2 = new QWidget(frame_5);
        verticalLayoutWidget_2->setObjectName(QString::fromUtf8("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 9, 134, 211));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        checkBox_2 = new QCheckBox(verticalLayoutWidget_2);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));

        verticalLayout_5->addWidget(checkBox_2);

        checkBox = new QCheckBox(verticalLayoutWidget_2);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));

        verticalLayout_5->addWidget(checkBox);

        checkBox_3 = new QCheckBox(verticalLayoutWidget_2);
        checkBox_3->setObjectName(QString::fromUtf8("checkBox_3"));

        verticalLayout_5->addWidget(checkBox_3);

        checkBox_4 = new QCheckBox(verticalLayoutWidget_2);
        checkBox_4->setObjectName(QString::fromUtf8("checkBox_4"));

        verticalLayout_5->addWidget(checkBox_4);

        checkBox_5 = new QCheckBox(verticalLayoutWidget_2);
        checkBox_5->setObjectName(QString::fromUtf8("checkBox_5"));

        verticalLayout_5->addWidget(checkBox_5);

        checkBox_6 = new QCheckBox(verticalLayoutWidget_2);
        checkBox_6->setObjectName(QString::fromUtf8("checkBox_6"));

        verticalLayout_5->addWidget(checkBox_6);

        tabWidget->addTab(tab_3, QString());
        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);
        pushButton->setDefault(true);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "FlightSimulator2024", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\">Flight Simulator 2024</p></body></html>", nullptr));
        label->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Left-Right</span></p></body></html>", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Up-Down</span></p></body></html>", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Throttle</span></p></body></html>", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "start", nullptr));
        pushButton_2->setText(QApplication::translate("MainWindow", "stop", nullptr));
        toolButton->setText(QApplication::translate("MainWindow", "...", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "height            :", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "acceleration :", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "velocity         :", nullptr));
        label_13->setText(QApplication::translate("MainWindow", "frequency    :", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Main", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "Moments:", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Total moments : ", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Acceleration      :", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "velocity               :", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Position              :", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "More", nullptr));
        groupBox->setTitle(QApplication::translate("MainWindow", "Graph", nullptr));
        pushButton_3->setText(QApplication::translate("MainWindow", "Print", nullptr));
        pushButton_4->setText(QApplication::translate("MainWindow", "clear", nullptr));
        checkBox_2->setText(QApplication::translate("MainWindow", "Height", nullptr));
        checkBox->setText(QApplication::translate("MainWindow", "Acceleration", nullptr));
        checkBox_3->setText(QApplication::translate("MainWindow", "velocity", nullptr));
        checkBox_4->setText(QApplication::translate("MainWindow", "Force x", nullptr));
        checkBox_5->setText(QApplication::translate("MainWindow", "Force y", nullptr));
        checkBox_6->setText(QApplication::translate("MainWindow", "force z", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "View", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // TEST1GVMRVN_H
