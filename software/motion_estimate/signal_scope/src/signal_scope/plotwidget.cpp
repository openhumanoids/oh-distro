#include "plotwidget.h"
#include "plot.h"
#include "lcmthread.h"
#include "signalhandler.h"
#include "setscaledialog.h"
#include "selectsignaldialog.h"

#include <qwt_scale_engine.h>
#include <qlabel.h>
#include <qlayout.h>
#include <QDoubleSpinBox>
#include <QMenu>
#include <QAction>
#include <QLabel>
#include <QInputDialog>
#include <QDialogButtonBox>

PlotWidget::PlotWidget(LCMThread* lcmThread, QWidget *parent):
    QWidget(parent),
    mLCMThread(lcmThread)
{

  d_plot = new Plot(this);

  mColors << Qt::green
          << Qt::red
          << Qt::blue
          << Qt::yellow
          << Qt::cyan
          << Qt::darkCyan;

  QDoubleSpinBox* timeWindowSpin = new QDoubleSpinBox;

  QVBoxLayout* vLayout1 = new QVBoxLayout();
  vLayout1->addWidget(timeWindowSpin);
  vLayout1->addWidget(new QLabel("Time Window [s]"));
  vLayout1->addStretch(10);

  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->addWidget(d_plot, 10);
  layout->addLayout(vLayout1);

  connect(timeWindowSpin, SIGNAL(valueChanged(double)),
          d_plot, SLOT(setIntervalLength(double)));
  timeWindowSpin->setValue(10.0);

  this->setContextMenuPolicy(Qt::CustomContextMenu);
  this->connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),
      SLOT(onShowContextMenu(const QPoint&)));

  this->start();
}

void PlotWidget::onShowContextMenu(const QPoint& pos)
{

  QPoint globalPos = this->mapToGlobal(pos);
  // for QAbstractScrollArea and derived classes you would use:
  // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); 

  QMenu myMenu;
  myMenu.addAction("Add signal");
  myMenu.addAction("Set Y axis scale");
  myMenu.addSeparator();
  myMenu.addAction("Remove plot");

  QAction* selectedItem = myMenu.exec(globalPos);
  if (!selectedItem)
  {
    return;
  }

  QString selectedAction = selectedItem->text();

  if (selectedAction == "Remove plot")
  {
    emit this->removePlotRequested(this);
  }
  else if (selectedAction == "Add signal")
  {
    this->addSignal();
  }
  else if (selectedAction == "Set Y axis scale")
  {

    SetScaleDialog dialog(this);
    QwtInterval axisInterval = d_plot->axisInterval(QwtPlot::yLeft);
    dialog.setUpper(axisInterval.maxValue());
    dialog.setLower(axisInterval.minValue());

    int result = dialog.exec();
    if (result == QDialog::Accepted)
    {
      d_plot->setAxisScale(QwtPlot::yLeft, dialog.lower(), dialog.upper());
    }
  }
}

void PlotWidget::start()
{
  d_plot->start();
}

void PlotWidget::addSignal()
{
  SelectSignalDialog dialog(this);
  int result = dialog.exec();
  if (result == QDialog::Accepted)
  {
    SignalHandler* handler = dialog.createSignalHandler();
    if (handler)
    {
      int signalColorIndex = mSignals.size() % mColors.size();
      mLCMThread->addSignalHandler(handler);
      mSignals.append(handler);
      d_plot->addSignal(handler->signalData(), mColors[signalColorIndex]);
    }
  }
}
