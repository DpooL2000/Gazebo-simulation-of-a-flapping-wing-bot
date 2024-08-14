#include <QApplication>
#include <QMainWindow>
#include <QSlider>
#include <QPushButton>
#include <QTextBrowser>
#include <QLabel>
#include <QKeyEvent>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <std_srvs/Empty.h>
#include "ui_test1.h"
#include <QPropertyAnimation>
#include <vector>
#include <QVector>
#include <QTimer>
#include <fstream>
#include <sstream>
#include "qcustomplot.h"
#include <QToolButton> 

class ColorButton : public QPushButton 
{
    Q_OBJECT

    Q_PROPERTY(QColor color READ color WRITE setColor)

    public:
        ColorButton(QWidget *parent = nullptr) : QPushButton(parent) {
            setColor(QColor("white"));
        }

        QColor color() const {
            return m_color;
        }

        void setColor(const QColor &color) {
            m_color = color;
            setStyleSheet(QString("background-color: %1;").arg(color.name()));
        }

    private:
        QColor m_color;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public: MainWindow(QWidget *parent = nullptr) : QMainWindow(parent)
    {
        ui.setupUi(this);
        nh = new ros::NodeHandle();

        // Apply dark mode stylesheet
        qApp->setStyleSheet(darkStyleSheet);

        // publishers for ROS topics
        this->freq_pub = this->nh->advertise<std_msgs::Float32>("/bird2/frequency", 10);
        this->pos_pub = this->nh->advertise<std_msgs::Float32>("/bird2/position", 10);
        this->dir_pub = this->nh->advertise<std_msgs::Float32>("/bird2/direction", 10);
        this->stat_pub = this->nh->advertise<std_msgs::Bool>("/bird2/states", 10);

        // subscribers for ROS topics 
        this->l4_sub = this->nh->subscribe("/bird2/t4", 1, &MainWindow::updateL4Torque, this);
        this->freq_sub = this->nh->subscribe("/bird2/frequency", 10, &MainWindow::updateFrequencyDisplay, this);
        this->height_sub = this->nh->subscribe("/bird2/height", 10, &MainWindow::updateHeight, this);
        this->velocity_sub = this->nh->subscribe("/bird2/velocity", 10, &MainWindow::updateVelocity, this);
        this->acceleration_sub = this->nh->subscribe("/bird2/acceleration", 10, &MainWindow::updateAcceleration, this);

        this->ui.horizontalSlider->setRange(-100, 100);
        this->ui.horizontalSlider->setValue(0);
        this->ui.verticalSlider_2->setRange(-100, 100);
        this->ui.verticalSlider_2->setValue(0);

        // all this for two damn colors!!
        this->colorButton1 = new ColorButton(this);
        this->colorButton2 = new ColorButton(this);
        this->animation1 = new QPropertyAnimation(colorButton1, "color");
        this->animation2 = new QPropertyAnimation(colorButton2, "color");
        this->colorButton1->setGeometry(this->ui.pushButton->geometry());
        this->colorButton2->setGeometry(this->ui.pushButton_2->geometry());
        this->colorButton1->setText(this->ui.pushButton->text());
        this->colorButton2->setText(this->ui.pushButton_2->text());
        this->ui.verticalLayout->addWidget(colorButton1);
        this->ui.verticalLayout->addWidget(colorButton2);
        this->ui.pushButton->hide();
        this->ui.pushButton_2->hide();
        this->colorButton1->show();
        this->colorButton2->show();

        QCustomPlot *customPlot = new QCustomPlot(this);
        QVBoxLayout *layout = new QVBoxLayout(ui.widget_2);
        layout->addWidget(customPlot);

        connect(ui.verticalSlider, &QSlider::valueChanged, this, &MainWindow::updateFrequency);
        connect(ui.verticalSlider_2, &QSlider::valueChanged, this, &MainWindow::updatePosition);
        connect(ui.horizontalSlider, &QSlider::valueChanged, this, &MainWindow::updatedirection);
        connect(ui.horizontalSlider, &QSlider::sliderReleased, this, &MainWindow::sliderReleased_2);
        connect(ui.verticalSlider_2, &QSlider::sliderReleased, this, &MainWindow::sliderReleased_1);
        connect(colorButton1, &QPushButton::clicked, this, &MainWindow::start);
        connect(colorButton2, &QPushButton::clicked, this, &MainWindow::stop);
        //connect(colorButton2, &QPushButton::clicked, this, &MainWindow::plotData);
        connect(ui.toolButton, &QToolButton::clicked, this, &MainWindow::showHelp);
        connect(colorButton1, &QPushButton::clicked, this, &MainWindow::green);
        connect(colorButton2, &QPushButton::clicked, this, &MainWindow::red);

        connect(ui.checkBox_2, &QCheckBox::toggled, this, &MainWindow::onCheckBoxToggled);
        connect(ui.checkBox, &QCheckBox::toggled, this, &MainWindow::onCheckBoxToggled);
        connect(ui.checkBox_3, &QCheckBox::toggled, this, &MainWindow::onCheckBoxToggled);
        connect(ui.pushButton_3, &QPushButton::clicked, this, [this]{
            QString filename = QFileDialog::getSaveFileName(this, "Save CSV", "", "CSV Files (*.csv)");
            if (!filename.isEmpty()) {
                saveDataToCSV(filename);
            }
        });

        ROS_INFO("GUI LANUCHED");
    }

    public slots:
        void updateFrequency(int value)
        {
            std_msgs::Float32 msg;
            msg.data = static_cast<int>(value * 8 / 100);
            this->freq_pub.publish(msg);
        }

        void updatePosition(int value)
        {
            std_msgs::Float32 msg;
            msg.data = static_cast<int>(value * 1.4 / 100);
            this->pos_pub.publish(msg);
        }

        void updatedirection(int value)
        {
            std_msgs::Float32 msg;
            msg.data = static_cast<int>(value * 1.4 / 100);
            this->dir_pub.publish(msg);
        }

        void sliderReleased_1()
        {
            this->ui.verticalSlider_2->setValue(0);
        }

        void sliderReleased_2()
        {
            this->ui.horizontalSlider->setValue(0);
        }

        void collectData()
        {
            if (latestHeightReceived && latestAccelerationReceived && latestVelocityReceived) {
                heightData.push_back(latestHeight);
                accelerationData.push_back(latestAcceleration);
                velocityData.push_back(latestVelocity);

                latestHeightReceived = false;
                latestAccelerationReceived = false;
                latestVelocityReceived = false;
            }
        }

        void start()
        {
            std_msgs::Bool msg;
            std_srvs::Empty srv;
            msg.data = static_cast<bool>(true);
            ros::service::call("/gazebo/unpause_physics", srv);
            this->stat_pub.publish(msg);

            // for collecting the simulation data
            dataCollectionTimer = new QTimer(this);
            connect(dataCollectionTimer, &QTimer::timeout, this, &MainWindow::collectData);
            dataCollectionTimer->start(100); 
        }

        void stop()
        {
            std_msgs::Bool msg;
            std_srvs::Empty srv;
            msg.data = static_cast<bool>(false);
            ros::service::call("/gazebo/pause_physics", srv);
            this->stat_pub.publish(msg);
            ROS_WARN("Gazebo simulation paused");
            
            // stop collectiong the data
            if (dataCollectionTimer) {
                dataCollectionTimer->stop();
                delete dataCollectionTimer;
                dataCollectionTimer = nullptr;
            }
        }

        void showHelp()
        {
            QMessageBox::information(this, "Help",
                "Controlls:\n\n"
                "Increase flap \t-- Up\n"
                "Decrease flap \t-- Down\n"
                "Turn left \t\t-- A\n"
                "Turn right \t-- D\n"
                "Pitch up \t\t-- W\n"
                "Pitch down \t-- S\n",
                QMessageBox::Ok);
        }

        void plotData()
        {
            customPlot->clearGraphs();
            
            QVector<double> x(heightData.size()), yHeight(heightData.size()), yAccelX(heightData.size()), yAccelY(heightData.size()), 
                                                                    yAccelZ(heightData.size()), yVelX(heightData.size()), yVelY(heightData.size()), yVelZ(heightData.size());

            for (size_t i = 0; i < heightData.size(); ++i)
            {
                x[i] = i;
                yHeight[i] = heightData[i];
                yAccelX[i] = accelerationData[i].x;
                yAccelY[i] = accelerationData[i].y;
                yAccelZ[i] = accelerationData[i].z;
                yVelX[i] = velocityData[i].x;
                yVelY[i] = velocityData[i].y;
                yVelZ[i] = velocityData[i].z;
            }

            customPlot->addGraph();
            customPlot->graph(0)->setData(x, yHeight);
            customPlot->graph(0)->setName("Height");

            customPlot->addGraph();
            customPlot->graph(1)->setData(x, yAccelX);
            customPlot->graph(1)->setName("Acceleration X");
            
            customPlot->addGraph();
            customPlot->graph(2)->setData(x, yAccelY);
            customPlot->graph(2)->setName("Acceleration Y");

            customPlot->addGraph();
            customPlot->graph(3)->setData(x, yAccelZ);
            customPlot->graph(3)->setName("Acceleration Z");

            customPlot->addGraph();
            customPlot->graph(4)->setData(x, yVelX);
            customPlot->graph(4)->setName("Velocity X");

            customPlot->addGraph();
            customPlot->graph(5)->setData(x, yVelY);
            customPlot->graph(5)->setName("Velocity Y");

            customPlot->addGraph();
            customPlot->graph(6)->setData(x, yVelZ);
            customPlot->graph(6)->setName("Velocity Z");

            customPlot->legend->setVisible(true);
            customPlot->rescaleAxes();
            customPlot->replot();
        }

        void onCheckBoxToggled(bool checked) 
        {
            QCheckBox *senderCheckBox = qobject_cast<QCheckBox *>(sender());
            if (senderCheckBox) 
            {  
                if (senderCheckBox == ui.checkBox_2) 
                {
                    if (checked) 
                        this->plotH = true;
                    else 
                        this->plotH = false;
                } 
                else if (senderCheckBox == ui.checkBox)
                {
                    if (checked) 
                        this->plotV = true;
                       
                    else 
                        this->plotV = false;
                }
                else if (senderCheckBox == ui.checkBox_3)
                {
                    if (checked) 
                        this->plotA = true;
                    else 
                        this->plotA = false;
                }  
            }
        }

        void saveDataToCSV(const QString &filename)
        {
            std::ofstream file(filename.toStdString());

            file << "Time";
            if (this->plotH)
                file << ",Height";

            if (this->plotA)
                file << ",Accel_X,Accel_Y,Accel_Z";

            if (this->plotV)
                file << ",Vel_X,Vel_Y,Vel_Z";

            file << "\n";

            for (int i = 0; i < heightData.size(); ++i)
            {
                file << i;

                if (this->plotH)
                    file << "," << heightData[i];

                if (this->plotA)
                    file << "," << accelerationData[i].x << "," 
                                << accelerationData[i].y << "," 
                                << accelerationData[i].z;
                if (this->plotV)
                    file << "," << velocityData[i].x << ","
                                << velocityData[i].y << "," 
                                << velocityData[i].z;
                file << "\n";
            }
            file.close();
        }

        void green()
        {
            this->animation1->setDuration(200);
            this->animation1->setStartValue("white");
            this->animation1->setEndValue("green");
            this->animation1->start();
            if (this->stateRed)
            {
                this->animation2->setDuration(200);
                this->animation2->setStartValue("red");
                this->animation2->setEndValue("white");
                this->animation2->start();
                this->stateRed = false;
            }
            this->stateGreen = true;
        }

        void red()
        {
            this->animation2->setDuration(200);
            this->animation2->setStartValue("white");
            this->animation2->setEndValue("red");
            this->animation2->start();
            if (this->stateGreen)
            {
                this->animation1->setDuration(200);
                this->animation1->setStartValue("green");
                this->animation1->setEndValue("white");
                this->animation1->start();
                this->stateGreen = false;
            }
            this->stateRed = true;
        }

        void keyPressEvent(QKeyEvent *event)
        {
            switch (event->key())
            {
                case Qt::Key_A:
                    this->ui.horizontalSlider->setValue(-100);
                    break;

                case Qt::Key_D:
                    this->ui.horizontalSlider->setValue(100);
                    break;

                case Qt::Key_W:
                    this->ui.verticalSlider_2->setValue(100);
                    break;

                case Qt::Key_S:
                    this->ui.verticalSlider_2->setValue(-100);
                    break;
                
                case Qt::Key_Up:
                    this->ui.verticalSlider->setValue(ui.verticalSlider->value() + 40);
                    break;

                case Qt::Key_Down:
                    this->ui.verticalSlider->setValue(ui.verticalSlider->value() - 40);
                    break;

                default:
                    QWidget::keyPressEvent(event);
                    break;
            }
        }

        void keyReleaseEvent(QKeyEvent *event)
        {
            switch (event->key())
            {
                case Qt::Key_A:
                    emit this->ui.horizontalSlider->sliderReleased();
                    break;

                case Qt::Key_D:
                    emit this->ui.horizontalSlider->sliderReleased();
                    break;

                case Qt::Key_W:
                    emit this->ui.verticalSlider_2->sliderReleased();
                    break;

                case Qt::Key_S:
                    emit this->ui.verticalSlider_2->sliderReleased();
                    break;

                default:
                    QWidget::keyPressEvent(event);
                    break;
            }
        }

        // slots to handle ROS messages and update QTextBrowser widgets
        void updateTorque4(const QString &textX, const QString &textY, const QString &textZ)
        {
            ui.textBrowser_4->setText(textX + "i");
            ui.textBrowser_5->setText(textY + "j");
            ui.textBrowser_6->setText(textZ + "k");
        }

        void updateL4Torque(const geometry_msgs::Vector3::ConstPtr &msg)
        {
            QString textX = QString::number(msg->x, 'f', 4);
            QString textY = QString::number(msg->y, 'f', 4);
            QString textZ = QString::number(msg->z, 'f', 4);

            QMetaObject::invokeMethod(this, "updateTorque4", Qt::QueuedConnection, Q_ARG(QString, textX), 
                                                                                   Q_ARG(QString, textY), 
                                                                                   Q_ARG(QString, textZ));
        }

        void updateVelocity(const geometry_msgs::Vector3::ConstPtr &velocity)
        {
            double velMagnitude = std::sqrt(velocity->x * velocity->x +
                                            velocity->y * velocity->y +
                                            velocity->z * velocity->z);

            QString velText = QString("%1 ms^-1").arg(velMagnitude, 0, 'f', 2);
            QString velText1 = QString("%1 i").arg(velocity->x, 0, 'f', 2);
            QString velText2 = QString("%1 j").arg(velocity->y, 0, 'f', 2);
            QString velText3 = QString("%1 k").arg(velocity->z, 0, 'f', 2);

            this->latestVelocity = *velocity;
            this->latestVelocityReceived = true;;

            QMetaObject::invokeMethod(this, "updateVelocityText", Qt::QueuedConnection, Q_ARG(QString, velText), 
                                                                                        Q_ARG(QString, velText1), 
                                                                                        Q_ARG(QString, velText2), 
                                                                                        Q_ARG(QString, velText3));
        }

        void updateVelocityText(const QString &text, const QString &text1, const QString &text2, const QString &text3)
        {
            ui.textBrowser->setText(text);
            ui.textBrowser_10->setText(text1);
            ui.textBrowser_11->setText(text2);
            ui.textBrowser_12->setText(text3);
        }

        void updateAcceleration(const geometry_msgs::Vector3::ConstPtr &acceleration)
        {
            double accMagnitude = std::sqrt(acceleration->x * acceleration->x +
                                            acceleration->y * acceleration->y +
                                            acceleration->z * acceleration->z);

            QString accText = QString("%1 ms^-2").arg(accMagnitude, 0, 'f', 2);
            QString accText1 = QString("%1 i").arg(acceleration->x, 0, 'f', 2);
            QString accText2 = QString("%1 j").arg(acceleration->y, 0, 'f', 2);
            QString accText3 = QString("%1 k").arg(acceleration->z, 0, 'f', 2);

            this->latestAcceleration = *acceleration;
            this->latestAccelerationReceived = true;

            QMetaObject::invokeMethod(this, "updateAccelerationText", Qt::QueuedConnection, Q_ARG(QString, accText), 
                                                                                            Q_ARG(QString, accText1), 
                                                                                            Q_ARG(QString, accText1), 
                                                                                            Q_ARG(QString, accText3));
        }

        void updateAccelerationText(const QString &text, const QString &text1, const QString &text2, const QString &text3)
        {
            ui.textBrowser_3->setText(text);
            ui.textBrowser_7->setText(text1);
            ui.textBrowser_8->setText(text2);
            ui.textBrowser_9->setText(text3);
        }

        void updateHeight(const std_msgs::Float32::ConstPtr &height)
        {
            QString heightText = QString("%1 m").arg(height->data, 0, 'f', 4);

            this->latestHeight = height->data;
            this->latestHeightReceived = true;

            QMetaObject::invokeMethod(this, "updateHeightText", Qt::QueuedConnection, Q_ARG(QString, heightText));
        }

        void updateHeightText(const QString &text)
        {
            ui.textBrowser_2->setText(text);
            ui.textBrowser_13->setText(text);

        }

        void updateFrequencyDisplay(const std_msgs::Float32::ConstPtr &frequency)
        {
            QString frequencyText = QString("%1 Hz").arg(frequency->data, 0, 'f', 2);;
            QMetaObject::invokeMethod(this, "updateFrequencyText", Qt::QueuedConnection, Q_ARG(QString, frequencyText));
        }

        void updateFrequencyText(const QString &text)
        {
            ui.textBrowser_16->setText(text);
        }   

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

#include "flight_controller_gui.moc"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flight_simulator_gui");

    QApplication app(argc, argv);
    app.setStyleSheet(darkStyleSheet);
    MainWindow w;
    w.show();

    std::thread rosThread([] {
        ros::spin();
    });

    int result = app.exec();

    rosThread.join();

    return result;
}