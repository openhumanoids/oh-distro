#include "setscaledialog.h"
#include "ui_setscaledialog.h"


class SetScaleDialog::Internal : public Ui::SetScaleDialog
{
public:

};

SetScaleDialog::SetScaleDialog(QWidget* parent) : QDialog(parent)
{
  mInternal = new Internal;
  mInternal->setupUi(this);
}

SetScaleDialog::~SetScaleDialog()
{
  delete mInternal;
}

double SetScaleDialog::lower() const
{
  return mInternal->LowerSpinBox->value();
}

double SetScaleDialog::upper() const
{
  return mInternal->UpperSpinBox->value();
}

void SetScaleDialog::setUpper(double upper)
{
  mInternal->UpperSpinBox->setValue(upper);
}

void SetScaleDialog::setLower(double lower)
{
  mInternal->LowerSpinBox->setValue(lower);
}
