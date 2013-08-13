#include "selectsignaldialog.h"
#include "ui_selectsignaldialog.h"

#include "signaldescription.h"
#include "signalhandler.h"
#include "jointnames.h"

#include <cstdio>
#include <cassert>

class SelectSignalDialog::Internal : public Ui::SelectSignalDialog
{
public:

};

SelectSignalDialog::SelectSignalDialog(QWidget* parent) : QDialog(parent)
{
  mInternal = new Internal;
  mInternal->setupUi(this);

  QStringList channels;
  channels << "ATLAS_STATE" << "ATLAS_STATUS" << "TRUE_ROBOT_STATE" << "EST_ROBOT_STATE";

  QStringList messageTypes = SignalHandlerFactory::instance().messageTypes();
  QStringList messageFields;

  mInternal->ChannelListBox->addItems(channels);
  mInternal->MessageTypeListBox->addItems(messageTypes);
  mInternal->MessageFieldListBox->addItems(messageFields);
  mInternal->JointNameListBox->addItems(JointNames::jointNames());

  mInternal->ChannelListBox->setCurrentRow(0);
  mInternal->MessageTypeListBox->setCurrentRow(0);
  mInternal->MessageFieldListBox->setCurrentRow(0);
  mInternal->JointNameListBox->setCurrentRow(0);

  this->connect(mInternal->MessageTypeListBox, SIGNAL(currentRowChanged(int)), SLOT(onMessageTypeChanged()));
  this->connect(mInternal->MessageFieldListBox, SIGNAL(currentRowChanged(int)), SLOT(onFieldNameChanged()));
  this->onMessageTypeChanged();
}

void SelectSignalDialog::onMessageTypeChanged()
{
  if (!mInternal->MessageTypeListBox->currentItem())
  {
    return;
  }

  QString messageType = mInternal->MessageTypeListBox->currentItem()->text();

  QStringList fieldNames = SignalHandlerFactory::instance().fieldNames(messageType);
  fieldNames.sort();

  mInternal->MessageFieldListBox->clear();
  mInternal->MessageFieldListBox->addItems(fieldNames);
  mInternal->MessageFieldListBox->setCurrentRow(0);
  this->onFieldNameChanged();
}


void SelectSignalDialog::onFieldNameChanged()
{
  if (!mInternal->MessageFieldListBox->currentItem())
  {
    return;
  }

  QString fieldName = mInternal->MessageFieldListBox->currentItem()->text();

  bool jointNamesVisible = fieldName.startsWith("joint_");

  mInternal->JointNameLabel->setVisible(jointNamesVisible);
  mInternal->JointNameListBox->setVisible(jointNamesVisible);
  mInternal->JointNameListBox->setVisible(jointNamesVisible);
}


SelectSignalDialog::~SelectSignalDialog()
{
  delete mInternal;
}

SignalHandler* SelectSignalDialog::createSignalHandler() const
{
  QString channel = mInternal->ChannelListBox->currentItem()->text();
  QString messageType = mInternal->MessageTypeListBox->currentItem()->text();
  QString messageField = mInternal->MessageFieldListBox->currentItem()->text();

  QString jointName;
  if (mInternal->JointNameListBox->currentItem())
  {
    jointName = mInternal->JointNameListBox->currentItem()->text();
  }

  SignalDescription desc;
  desc.mChannel = channel;
  desc.mMessageType = messageType;
  desc.mFieldName = messageField;
  desc.mArrayKeys.append(jointName);

  SignalHandler* signalHandler = SignalHandlerFactory::instance().createHandler(&desc);
  assert(signalHandler);
  return signalHandler;
}

