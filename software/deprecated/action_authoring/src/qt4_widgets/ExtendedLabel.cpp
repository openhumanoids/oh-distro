#include "ExtendedLabel.h"

ExtendedQLabel::
ExtendedQLabel(const QString &text, QWidget *parent):
    QLabel(parent)
{
    this->setText(text);
}

ExtendedQLabel::
~ExtendedQLabel()
{

}

void
ExtendedQLabel
::mouseMoveEvent(QMouseEvent *e)
{
    emit hover();
}

void
ExtendedQLabel
::mouseReleaseEvent(QMouseEvent *e)
{
    emit clicked();
}

