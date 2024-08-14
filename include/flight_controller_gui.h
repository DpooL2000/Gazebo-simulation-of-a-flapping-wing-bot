#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QTextBrowser>
#include <QLabel>
#include <QKeyEvent>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <vector>
#include <QPropertyAnimation>
#include <QTimer>
#include "qcustomplot.h"

class ColorButton : public QPushButton 
{
    Q_OBJECT
    Q_PROPERTY(QColor color READ color WRITE setColor)

public:
    explicit ColorButton(QWidget *parent = nullptr);
    QColor color() const;
    void setColor(const QColor &color);

private:
    QColor m_color;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

public slots:
    void updateFrequency(int value);
    void updatePosition(int value);
    void updatedirection(int value);
    void sliderReleased_1();
    void sliderReleased_2();
    void collectData();
    void start();
    void stop();
    void showHelp();
    void plotData();
    void onCheckBoxToggled(bool checked);
    void saveDataToCSV(const QString &filename);
    void green();
    void red();
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;

    void updateTorque4(const QString &textX, const QString &textY, const QString &textZ);
    void updateL4Torque(const geometry_msgs::Vector3::ConstPtr &msg);
    void updateVelocity(const geometry_msgs::Vector3::ConstPtr &velocity);
    void updateVelocityText(const QString &text, const QString &text1, const QString &text2, const QString &text3);
    void updateAcceleration(const geometry_msgs::Vector3::ConstPtr &acceleration);
    void updateAccelerationText(const QString &text, const QString &text1, const QString &text2, const QString &text3);
    void updateHeight(const std_msgs::Float32::ConstPtr &height);
    void updateHeightText(const QString &text);
    void updateFrequencyDisplay(const std_msgs::Float32::ConstPtr &frequency);
    void updateFrequencyText(const QString &text);

private:
    Ui::MainWindow ui;
    ros::NodeHandle* nh;
    ros::Publisher freq_pub;
    ros::Publisher pos_pub;
    ros::Publisher dir_pub;
    ros::Publisher stat_pub;
    ros::Subscriber l4_sub;
    ros::Subscriber freq_sub;
    ros::Subscriber height_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber acceleration_sub;
    QPropertyAnimation *animation1;
    QPropertyAnimation *animation2;
    ColorButton *colorButton1;
    ColorButton *colorButton2;
    QWidget *plotTab;
    QCustomPlot *customPlot;
    bool stateRed = false;
    bool stateGreen = false;

    std::vector<float> heightData;
    std::vector<geometry_msgs::Vector3> accelerationData;
    std::vector<geometry_msgs::Vector3> velocityData;
    float latestHeight;
    geometry_msgs::Vector3 latestAcceleration;
    geometry_msgs::Vector3 latestVelocity;
    bool latestHeightReceived;
    bool latestAccelerationReceived;
    bool latestVelocityReceived;
    QTimer *dataCollectionTimer;
    bool plotH = false;
    bool plotV = false;
    bool plotA = false;
};

#endif 
