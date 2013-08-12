#ifndef HOMING_DIALOG_H
#define HOMING_DIALOG_H

#include <QDialog>
#include <QTimer>
#include <QScrollBar>
#include <ros/ros.h>
#include <sandia_hand_msgs/SetJointLimitPolicy.h>
#include <sandia_hand_msgs/CalFingerState.h>
#include <sandia_hand_msgs/RelativeJointCommands.h>

class QTabWidget;

class ManualFingerSubtab : public QWidget
{
  Q_OBJECT
public:
  ManualFingerSubtab(QWidget *parent, 
                     ros::NodeHandle &nh,
                     ros::Publisher *finger_joint_commands_pub,
                     const int finger_idx);
  static double SLIDER_TICKS_PER_DEGREE;
  QPushButton *home_button_;
  QScrollBar *sb_[3];
  ros::NodeHandle nh_;
  ros::Publisher *finger_joint_commands_pub_;
  int finger_idx_;
  bool is_resetting_;
public slots:
  void sendFingerPose();
  void setFingerHome();
};

class ManualTab : public QWidget
{
  Q_OBJECT
public:
  ManualTab(QWidget  *parent, ros::NodeHandle &nh, 
            ros::Publisher *finger_pubs);
  QTabWidget         *tabs_;
  ManualFingerSubtab *finger_tabs_[4];
  ros::NodeHandle     nh_;
  ros::Publisher     *finger_pubs_[4];
};

class AutoTab : public QWidget
{
  Q_OBJECT
public:
  AutoTab(QWidget *parent, ros::NodeHandle &nh, ros::Publisher *finger_pub);
  QPushButton *automove_button_, *home_button_, *move_thumb_button_;
private:
  ros::NodeHandle   nh_;
  ros::Publisher   *relative_finger_pub_;
  ros::Subscriber   cal_finger_status_subs_[4];
  void cal_finger_state_cb(
                      const uint8_t finger_idx, 
                      const sandia_hand_msgs::CalFingerState::ConstPtr &msg);
  QTimer *ros_update_timer_;
  bool homing_enabled_;
  double last_homing_time_;
  bool status_rx_complete_[4];
  sandia_hand_msgs::RelativeJointCommands rjc_;
public slots:
  void automove(bool enabled);
  void home();
  void move_thumb();
  void rosTimerTimeout();
};

class HomingDialog : public QDialog
{
  Q_OBJECT
public:
  HomingDialog(QWidget *parent = 0);
  virtual ~HomingDialog();
  ros::ServiceClient set_joint_policy_client_;
  ros::Publisher     finger_pubs_[4], relative_finger_pub_;
private:
  QTabWidget       *tabs_;
  ros::NodeHandle   nh_;
};

#endif

