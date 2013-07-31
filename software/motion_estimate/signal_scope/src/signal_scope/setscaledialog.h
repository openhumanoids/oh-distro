#ifndef _SETSCALEDIALOG_H_
#define _SETSCALEDIALOG_H_

#include <QDialog>

class SetScaleDialog : public QDialog
{
  Q_OBJECT
public:

  SetScaleDialog(QWidget* parent=0);
  virtual ~SetScaleDialog();

  double lower() const;
  double upper() const;

  void setLower(double lower);
  void setUpper(double upper);

private:

  class Internal;
  Internal* mInternal;
};

#endif
