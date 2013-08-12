#include <QApplication>
#include <ros/ros.h>
#include "homing_dialog.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sandia_hand_homing_gui");
  QApplication app(argc, argv);
  HomingDialog hd;
  hd.show();
  return app.exec();
}
