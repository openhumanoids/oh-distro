#ifndef _SELECTSIGNALDIALOG_H_
#define _SELECTSIGNALDIALOG_H_

#include <QDialog>

class SignalHandler;

class SelectSignalDialog : public QDialog
{
  Q_OBJECT
public:

  SelectSignalDialog(QWidget* parent=0);
  virtual ~SelectSignalDialog();

  QList<SignalHandler*> createSignalHandlers() const;

protected slots:

  void onMessageTypeChanged();
  void onFieldNameChanged();

private:

  class Internal;
  Internal* mInternal;
};

#endif
