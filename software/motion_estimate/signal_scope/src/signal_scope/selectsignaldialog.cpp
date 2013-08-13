#include "selectsignaldialog.h"
#include "ui_selectsignaldialog.h"

#include "signaldescription.h"
#include "signalhandler.h"

class SelectSignalDialog::Internal : public Ui::SelectSignalDialog
{
public:

};

SelectSignalDialog::SelectSignalDialog(QWidget* parent) : QDialog(parent)
{
  mInternal = new Internal;
  mInternal->setupUi(this);

  QStringList channels;
  channels << "TRUE_ROBOT_STATE" << "EST_ROBOT_STATE";

  QStringList messageTypes;
  messageTypes << "drc.robot_state_t";

  QStringList messageFields;
  messageFields << "joint_position"
                << "joint_velocity"
                << "joint_effort";

  mInternal->ChannelListBox->addItems(channels);
  mInternal->MessageTypeListBox->addItems(messageTypes);
  mInternal->MessageFieldListBox->addItems(messageFields);

  mInternal->ChannelListBox->setCurrentRow(0);
  mInternal->MessageTypeListBox->setCurrentRow(0);
  mInternal->MessageFieldListBox->setCurrentRow(0);
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
  int arrayIndex = mInternal->ArrayIndexSpinBox->value();
  //bool fieldIsArray = mInternal->FieldIsArrayCheckBox->isChecked();

  SignalDescription desc;
  desc.mChannel = channel;
  desc.mMessageType = messageType;
  desc.mFieldName = messageField;
  desc.mArrayKeys.append(QString::number(arrayIndex));

  return SignalHandlerFactory::instance().createHandler(&desc);
}

