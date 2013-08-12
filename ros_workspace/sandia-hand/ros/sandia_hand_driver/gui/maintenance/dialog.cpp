#include <QtGui>
#include <QMessageBox>
#include <QComboBox>
#include <QGroupBox>
#include "dialog.h"
#include <string>
#include <vector>
#include <utility>
using std::string;
using std::vector;
using std::pair;
using std::make_pair;

Dialog::Dialog(QWidget *parent)
: QDialog(parent)
{
  firmware_operation_ = "Update Firmware";
  tabWidget = new QTabWidget;
  tabWidget->addTab(new FirmwareTab(this), tr("Firmware"));
  tabWidget->addTab(new FunctionalTestTab(this), tr("Functional Test"));
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  //connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setSizeConstraint(QLayout::SetNoConstraint);
  mainLayout->addWidget(tabWidget);
  mainLayout->addWidget(buttonBox);
  setLayout(mainLayout);
  setWindowTitle(tr("Sandia Hand"));
}

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

FirmwareTab::FirmwareTab(QWidget *parent)
: QWidget(parent)
{
  QVBoxLayout *mainLayout   = new QVBoxLayout;
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
}

FunctionalTestTab::FunctionalTestTab(QWidget *parent)
: QWidget(parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addStretch(1);
  setLayout(mainLayout);
}

