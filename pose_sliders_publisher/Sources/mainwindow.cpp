#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle *nh, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    connect(ui->X_slider, SIGNAL(sliderReleased()), this, SLOT(SlidersCallback()));
    connect(ui->Y_slider, SIGNAL(sliderReleased()), this, SLOT(SlidersCallback()));
    connect(ui->Z_slider, SIGNAL(sliderReleased()), this, SLOT(SlidersCallback()));
    connect(ui->Roll_slider, SIGNAL(sliderReleased()), this, SLOT(SlidersCallback()));
    connect(ui->Pitch_slider, SIGNAL(sliderReleased()), this, SLOT(SlidersCallback()));
    connect(ui->Yaw_slider, SIGNAL(sliderReleased()), this, SLOT(SlidersCallback()));

    _pose_pub = nh->advertise<std_msgs::Float64MultiArray>("PoseSlidersCallback", 1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::SlidersCallback()
{
  _pose.x = static_cast<double>(ui->X_slider->value())/1000.0;
  _pose.y = static_cast<double>(ui->Y_slider->value())/1000.0;
  _pose.z = static_cast<double>(ui->Z_slider->value())/1000.0;
  _pose.roll = static_cast<double>(ui->Roll_slider->value())/1000.0;
  _pose.pitch = static_cast<double>(ui->Pitch_slider->value())/1000.0;
  _pose.yaw = static_cast<double>(ui->Yaw_slider->value())/1000.0;

  std_msgs::Float64MultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].label = "SlidersPose";
  msg.layout.dim[0].size = 6;
  msg.layout.dim[0].stride = 6;
  msg.data.resize(6);
  memcpy(&msg.data[0], &_pose, sizeof(_pose));
  _pose_pub.publish(msg);
}

void MainWindow::on_pushButton_clicked()
{
  ui->X_slider->setValue(307);
  ui->Y_slider->setValue(0);
  ui->Z_slider->setValue(487);
  ui->Roll_slider->setValue(-3142);
  ui->Pitch_slider->setValue(-1);
  ui->Yaw_slider->setValue(0);
  SlidersCallback();
}
