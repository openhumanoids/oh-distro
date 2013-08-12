#include <QtGui>
#include <QMessageBox>
#include <QComboBox>
#include <QGroupBox>
#include "homing_dialog.h"
#include <string>
#include <vector>
#include <utility>
#include <osrf_msgs/JointCommands.h>
#include <sandia_hand_msgs/SetFingerHome.h>
#include <boost/bind.hpp>
using std::string;
using std::vector;
using std::pair;
using std::make_pair;
double ManualFingerSubtab::SLIDER_TICKS_PER_DEGREE = 2;

HomingDialog::HomingDialog(QWidget *parent)
: QDialog(parent)
{
  set_joint_policy_client_ = 
    nh_.serviceClient<sandia_hand_msgs::SetJointLimitPolicy>
                     ("set_joint_limit_policy");
  sandia_hand_msgs::SetJointLimitPolicy srv;
  srv.request.policy = "none";
  if (!set_joint_policy_client_.call(srv))
  {
    ROS_FATAL("couldn't set joint limit policy.");
    QMetaObject::invokeMethod(this, "close", Qt::QueuedConnection); // bail
  }
  else
    ROS_INFO("Released joint limits. Be careful...");
  for (int i = 0; i < 4; i++)
  {
    char topic[100];
    snprintf(topic, sizeof(topic), "finger_%d/joint_commands", i);
    finger_pubs_[i] = nh_.advertise<osrf_msgs::JointCommands>(topic, 1);
  }
  relative_finger_pub_ =
    nh_.advertise<sandia_hand_msgs::RelativeJointCommands>
      ("relative_joint_commands", 1);
  tabs_ = new QTabWidget;
  tabs_->addTab(new ManualTab(this, nh_, finger_pubs_), tr("Manual"));
  tabs_->addTab(new AutoTab(this, nh_, &relative_finger_pub_), tr("Auto"));
  //button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok);
  //connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  //connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->setSizeConstraint(QLayout::SetNoConstraint);
  main_layout->addWidget(tabs_);
  //main_layout->addWidget(button_box_);
  main_layout->addStretch(1);
  setLayout(main_layout);
  setWindowTitle(tr("Homing"));
}

HomingDialog::~HomingDialog()
{
  sandia_hand_msgs::SetJointLimitPolicy srv;
  srv.request.policy = "default";
  if (!set_joint_policy_client_.call(srv))
    ROS_ERROR("couldn't restore joint limit policy.");
}

/*
void Dialog::onFirmwareOperationChanged(const QString &operation_qstr)
{
  string op = operation_qstr.toStdString();
  printf("onFirmwareOperationChanged(%s)\n", op.c_str());
  firmware_operation_ = op;
}

void Dialog::onFirmwareButton(const QString &board_name_qstr)
{
  string board = board_name_qstr.toStdString();
  printf("onFirmwareButton(%s)\n", board.c_str());
  string cmd;
  if (firmware_operation_.find("ootloader") != std::string::npos) // fix this
  {
    cmd = string("cd `rospack find sandia_hand_driver`/../../firmware/build && make ") + board + string("-bl-gpnvm && make ") + board + string("-bl-program");
    if (board == string("mobo"))
      cmd = string("cd `rospack find sandia_hand_driver`/../../firmware/build && make mobo-mcu-set_boot_vector && make mobo-mcu-program");
  }
  else
  {
    if (board == string("mobo"))
      cmd = string("cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mmcu_burn ../../firmware/build/mobo/mcu/mobo-mcu.bin");
    else if (board == string("fmcb"))
      cmd = string("cd `rospack find sandia_hand_driver` && bin/sandia_hand_cli mmburn 0 ../../firmware/build/fmcb/std/fmcb-std.bin"); // todo: finger_idx
    //cmd = string("cd `rospack find sandia_hand_driver` && bin/loose_finger_cli /dev/ttyUSB0 burn `rospack find sandia_hand_driver`/../../firmware/build/fmcb/std/fmcb-std.bin");
  }
  if (cmd.length() == 0)
  {
    QMessageBox::critical(this, tr("Operation currently not implemented"),
                          tr("Operation currently not implemented."));
    return;
  }

  //if (board == string("f2") || board == string("f3"))
  //  cmd = "cd `rospack find sandia_hand_driver`/cli/loose_finger_cli 
  printf("%s\n", cmd.c_str());
  int rv = system(cmd.c_str());
  if (rv)
  {
    QMessageBox::critical(this, tr("Firmware Load Error"),
                          tr("See terminal for details."));
    return;
  }
  QMessageBox::information(this, tr("Firmware Load Complete"),
                           tr("Successfully loaded firmware."));
}
*/

ManualFingerSubtab::ManualFingerSubtab(QWidget *parent,
                                    ros::NodeHandle &nh,
                                    ros::Publisher *finger_joint_commands_pub,
                                    const int finger_idx)
: QWidget(parent),
  nh_(nh),
  finger_joint_commands_pub_(finger_joint_commands_pub),
  finger_idx_(finger_idx),
  is_resetting_(false)
{
  QGridLayout *grid = new QGridLayout(this);  
  const char *row_labels[3] = { "Abduction/Adduction:", 
                                "Proximal Flexion:", 
                                "Distal Flexion:" };
  for (int i = 0; i < 3; i++)
  {
    grid->addWidget(new QLabel(row_labels[i]), i, 0, Qt::AlignRight);
    sb_[i] = new QScrollBar(Qt::Horizontal);
    sb_[i]->setFocusPolicy(Qt::StrongFocus);
    sb_[i]->setMinimum(-120 * SLIDER_TICKS_PER_DEGREE);
    sb_[i]->setMaximum( 120 * SLIDER_TICKS_PER_DEGREE);
    sb_[i]->setValue(0);
    grid->addWidget(sb_[i], i, 1);
    connect(sb_[i], SIGNAL(valueChanged(int)), this, SLOT(sendFingerPose())); 
  }
  grid->addWidget(home_button_ = new QPushButton(tr("Set Finger Home")), 
                  3, 1, Qt::AlignCenter);
  connect(home_button_, SIGNAL(clicked()), this, SLOT(setFingerHome()));
  grid->setColumnMinimumWidth(1, 200);
  setLayout(grid);
}

void ManualFingerSubtab::sendFingerPose()
{
  if (is_resetting_)
    return; // in the middle of returning all sliders to zero, after homing
  osrf_msgs::JointCommands jc;
  jc.position.resize(3);
  for (int i = 0; i < 3; i++)
    jc.position[i] = (double)sb_[i]->value() / SLIDER_TICKS_PER_DEGREE * 
                     3.14 / 180.0;
  //printf("finger pose: %.3f %.3f %.3f\n", 
  //       jc.position[0], jc.position[1], jc.position[2]);
  finger_joint_commands_pub_->publish(jc);
}

void ManualFingerSubtab::setFingerHome()
{
  ROS_INFO("ManualFingerSubtab::setFingerHome");
  ros::ServiceClient set_home_client = 
    nh_.serviceClient<sandia_hand_msgs::SetFingerHome>("set_finger_home");
  sandia_hand_msgs::SetFingerHome srv;
  srv.request.finger_idx = finger_idx_;
  if (!set_home_client.call(srv))
  {
    ROS_ERROR("couldn't set home");
    return;
  }
  ROS_INFO("set home");
  is_resetting_ = true;
  for (int i = 0; i < 3; i++)
    sb_[i]->setValue(0);
  is_resetting_ = false;
}

ManualTab::ManualTab(QWidget *parent, ros::NodeHandle &nh, 
                     ros::Publisher *finger_pubs)
: QWidget(parent),
  nh_(nh)
{
  for (int i = 0; i < 4; i++)
    finger_pubs_[i] = &finger_pubs[i];
  QVBoxLayout *main_layout = new QVBoxLayout;
  tabs_ = new QTabWidget;
  const char *tab_names[4] = { "Index", "Middle", "Pinkie", "Thumb" };
  for (int i = 0; i < 4; i++)
    tabs_->addTab(finger_tabs_[i] = 
                      new ManualFingerSubtab(this, nh_, finger_pubs_[i], i), 
                  tr(tab_names[i]));
  main_layout->addWidget(tabs_);
  main_layout->addStretch(1);
  setLayout(main_layout);
  //buttons.push_back(make_pair(new QPushButton(tr("&Distal Phalange")),"f3"));

/*
  vector< pair<QPushButton *,QString> > buttons;
  QComboBox *cb = new QComboBox; //(this);
  cb->addItem(tr("Update Firmware"));
  cb->addItem(tr("Install Bootloader"));
  QHBoxLayout *op_layout = new QHBoxLayout;
  op_layout->addWidget(new QLabel("Operation:"));
  op_layout->addWidget(cb);
  mainLayout->addLayout(op_layout);
  buttons.push_back(make_pair(new QPushButton(tr("&Distal Phalange")),"f3"));
  buttons.push_back(make_pair(new QPushButton(tr("&Proximal Phalange")),"f2"));
  buttons.push_back(make_pair(new QPushButton(tr("&Motor Board")),"fmcb"));
  buttons.push_back(make_pair(new QPushButton(tr("&Right Palm")),"rpalm"));
  buttons.push_back(make_pair(new QPushButton(tr("&Left Palm")),"lpalm"));
  buttons.push_back(make_pair(new QPushButton(tr("Mother&board")),"mobo"));
  button_mapper = new QSignalMapper(this);
  QGroupBox *gb = new QGroupBox(tr("Click Board Name to Perform Operation"));
  QVBoxLayout *gb_layout = new QVBoxLayout();
  for (size_t i = 0; i < buttons.size(); i++)
  {
    gb_layout->addWidget(buttons[i].first);
    button_mapper->setMapping(buttons[i].first, buttons[i].second);
    connect(buttons[i].first, SIGNAL(clicked()), button_mapper, SLOT(map()));
  }
  gb->setLayout(gb_layout);
  mainLayout->addWidget(gb);
  mainLayout->addStretch(1);
  setLayout(mainLayout);

  connect(button_mapper, SIGNAL(mapped(const QString &)),
          parent, SLOT(onFirmwareButton(const QString &)));
  connect(cb, SIGNAL(currentIndexChanged(const QString &)), 
          parent, SLOT(onFirmwareOperationChanged(const QString &)));
  */
}

AutoTab::AutoTab(QWidget *parent, ros::NodeHandle &nh, 
                 ros::Publisher *relative_finger_pub)
: QWidget(parent),
  nh_(nh),
  relative_finger_pub_(relative_finger_pub),
  homing_enabled_(false),
  last_homing_time_(0)
{
  for (int i = 0; i < 4; i++)
  {
    char topic_name[100];
    snprintf(topic_name, sizeof(topic_name), "finger_%d/cal_state", i);
    cal_finger_status_subs_[i] = 
      nh_.subscribe<sandia_hand_msgs::CalFingerState>(topic_name, 1, 
                    boost::bind(&AutoTab::cal_finger_state_cb, this, i, _1));
    status_rx_complete_[i] = false;
  }
  QVBoxLayout *main_layout = new QVBoxLayout;
  main_layout->addStretch(1);
  main_layout->addWidget(automove_button_ = new QPushButton(tr("Auto-Move")));
  main_layout->addWidget(home_button_ = new QPushButton(tr("Set Home")));
  main_layout->addWidget(move_thumb_button_ = 
                         new QPushButton(tr("Offset Thumb")));
  automove_button_->setCheckable(true);
  connect(move_thumb_button_, SIGNAL(clicked()), this, SLOT(move_thumb()));
  connect(home_button_, SIGNAL(clicked()), this, SLOT(home()));
  connect(automove_button_, SIGNAL(toggled(bool)), this, SLOT(automove(bool)));
  setLayout(main_layout);
  ros_update_timer_ = new QTimer(this);
  connect(ros_update_timer_, SIGNAL(timeout()), this, SLOT(rosTimerTimeout()));
  ros_update_timer_->start(1);
}

void AutoTab::rosTimerTimeout()
{
  ros::spinOnce();
}

inline static double clamp_mag(double d, const double mag)
{
  if (d > mag)
    return mag;
  else if (d < -mag)
    return -mag;
  return d;
}

void AutoTab::cal_finger_state_cb(
                       const uint8_t finger_idx,
                       const sandia_hand_msgs::CalFingerState::ConstPtr &msg)
{
  if (finger_idx >= 4)
    return;
  if (!homing_enabled_)
    return;
  //if (finger_idx != 0)
  //  return;
  double t = ros::Time::now().toSec();
  if (t - last_homing_time_ > 1.0)
  {
    status_rx_complete_[finger_idx] = true;
    const double max_move = 0.1;
    for (int i = 0; i < 3; i++)
    {
      if (msg->joints_inertial_variance[i] < 1)
      {
        double home_pos = 0;
        if (finger_idx == 3 && i == 0)
          home_pos = M_PI / 4; // avoid measurement singularity badness...
        rjc_.position[finger_idx*3+i] = 
          clamp_mag(-(msg->joints_inertial[i] - home_pos), max_move);
      }
    }
    bool all_ok = true;
    for (int i = 0; i < 4; i++)
      all_ok &= status_rx_complete_[i];
    if (all_ok)
    {
      for (int f = 0; f < 4; f++)
        printf("  %d: %.3f  %.3f  %.3f\n",
               f, 
               rjc_.position[f*3],
               rjc_.position[f*3+1],
               rjc_.position[f*3+2]);
      relative_finger_pub_->publish(rjc_);
      for (int i = 0; i < 12; i++)
      {
        rjc_.position[i] = 0;
        rjc_.max_effort[i] = 50;
      }
      for (int i = 0; i < 4; i++)
        status_rx_complete_[i] = false;
      last_homing_time_ = t;
    }
  }

  //printf("inertial encoding j2 for finger 0: %.3f\n", msg->joints_inertial[2]);
  //printf("finger status cb for finger %d\n", finger_idx);
}

void AutoTab::automove(bool enabled)
{
  printf("AutoTab::home(%d)\n", (int)enabled);
  homing_enabled_ = enabled;
}

void AutoTab::home()
{
  printf("set home\n");
  for (int finger_idx = 0; finger_idx < 4; finger_idx++)
  {
    ros::ServiceClient set_home_client = 
      nh_.serviceClient<sandia_hand_msgs::SetFingerHome>("set_finger_home");
    sandia_hand_msgs::SetFingerHome srv;
    srv.request.finger_idx = finger_idx;
    if (!set_home_client.call(srv))
    {
      ROS_ERROR("couldn't set finger %d home", finger_idx);
      return;
    }
    ROS_INFO("set home, finger %d", finger_idx);
  }
}

void AutoTab::move_thumb()
{
  if (homing_enabled_)
  {
    printf("couldn't move thumb since it's still trying to auto move\n");
    return;
  }
  printf("moving thumb\n");
  for (int i = 0; i < 12; i++)
    rjc_.position[i] = 0;
  rjc_.position[9] = -M_PI/4;
  relative_finger_pub_->publish(rjc_);
  for (int i = 0; i < 12; i++)
    rjc_.position[i] = 0;
}

