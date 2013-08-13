#include "plotwidget.h"
#include "plot.h"
#include "lcmthread.h"
#include "signalhandler.h"
#include "setscaledialog.h"
#include "selectsignaldialog.h"
#include "signaldescription.h"

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
  timeWindowSpin->setSingleStep(0.1);

  QDoubleSpinBox* yScaleSpin = new QDoubleSpinBox;
  yScaleSpin->setSingleStep(0.1);

  QVBoxLayout* vLayout1 = new QVBoxLayout();
  vLayout1->addWidget(timeWindowSpin);
  vLayout1->addWidget(new QLabel("Time Window [s]"));
  vLayout1->addWidget(yScaleSpin);
  vLayout1->addWidget(new QLabel("Y Scale [+/-]"));
  vLayout1->addStretch(10);

  QHBoxLayout *layout = new QHBoxLayout(this);
  layout->addWidget(d_plot, 10);
  layout->addLayout(vLayout1);

  connect(timeWindowSpin, SIGNAL(valueChanged(double)),
          d_plot, SLOT(setIntervalLength(double)));
  timeWindowSpin->setValue(10.0);

  connect(yScaleSpin, SIGNAL(valueChanged(double)),
          d_plot, SLOT(setYScale(double)));
  yScaleSpin->setValue(10.0);

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
    emit this->addSignalRequested(this);
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

void PlotWidget::stop()
{
  d_plot->stop();
}

void PlotWidget::addSignal(const QMap<QString, QVariant>& signalSettings)
{
  SignalDescription desc;
  desc.mChannel = signalSettings.value("channel").toString();
  desc.mMessageType = signalSettings.value("messageType").toString();
  desc.mFieldName = signalSettings.value("fieldName").toString();
  desc.mArrayKeys = signalSettings.value("arrayKeys").toStringList();

  SignalHandler* signalHandler = SignalHandlerFactory::instance().createHandler(&desc);

  if (signalHandler)
  {
    printf("adding signal: %s\n", qPrintable(signalHandler->description()));
  }
  else
  {
    printf("failed to create signal from signal settings.\n");
  }

  this->addSignal(signalHandler);
}

void PlotWidget::addSignal(SignalHandler* signalHandler)
{
  if (!signalHandler)
  {
    return;
  }

  int signalColorIndex = mSignals.size() % mColors.size();
  mLCMThread->addSignalHandler(signalHandler);
  mSignals.append(signalHandler);
  d_plot->addSignal(signalHandler->signalData(), mColors[signalColorIndex]);
}


void PlotWidget::loadSettings(const QMap<QString, QVariant>& plotSettings)
{
  QList<QVariant> signalList = plotSettings.value("signals").toList();
  foreach (const QVariant& signalVariant, signalList)
  {
    this->addSignal(signalVariant.toMap());
  }
}
