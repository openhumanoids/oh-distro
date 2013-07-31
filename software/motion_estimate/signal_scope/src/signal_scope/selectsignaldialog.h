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

  SignalHandler* createSignalHandler() const;

protected slots:

private:

  class Internal;
  Internal* mInternal;
};

#endif
