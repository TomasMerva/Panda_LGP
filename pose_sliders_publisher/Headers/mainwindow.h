#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

struct PoseStruct
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle *nh, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
  void SlidersCallback();

  void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    PoseStruct _pose;
    ros::Publisher _pose_pub;
};

#endif // MAINWINDOW_H
