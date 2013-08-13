#include "mainwindow.h"
#include "plotwidget.h"
#include "selectsignaldialog.h"
#include "signaldescription.h"
#include "lcmthread.h"

#include <qlabel.h>
#include <qlayout.h>
#include <QApplication>
#include <QDebug>
#include <QScrollArea>
#include <QPushButton>
#include <QShortcut>

#include "qjson.h"

#include <cstdio>

MainWindow::MainWindow(QWidget* parent): QWidget(parent)
{
  this->setWindowTitle("Signal Scope");

  mLCMThread = new LCMThread;
  mLCMThread->start();

  QVBoxLayout* vLayout = new QVBoxLayout(this);

  QWidget* toolbar = new QWidget;
  QHBoxLayout* hlayout = new QHBoxLayout(toolbar);

  QPushButton* newPlotButton = new QPushButton("Add plot");
  hlayout->addWidget(newPlotButton);

  QPushButton* pauseButton = new QPushButton("Pause");
  pauseButton->setCheckable(true);
  hlayout->addWidget(pauseButton);

  hlayout->addStretch();


  this->connect(newPlotButton, SIGNAL(clicked()), SLOT(onNewPlotClicked()));
  this->connect(pauseButton, SIGNAL(clicked()), SLOT(onTogglePause()));

  mScrollArea = new QScrollArea;
  mPlotArea = new QWidget;
  mPlotLayout = new QVBoxLayout(mPlotArea);

  mScrollArea->setWidget(mPlotArea);
  mScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  mScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  mScrollArea->setWidgetResizable(true);

  vLayout->addWidget(toolbar);
  vLayout->addWidget(mScrollArea);

 QShortcut* quit = new QShortcut(QKeySequence(tr("Ctrl+Q", "Quit")), this);
  this->connect(quit, SIGNAL(activated()), SLOT(close()));


  this->loadSettings();
}

MainWindow::~MainWindow()
{
  mLCMThread->stop();
  mLCMThread->wait(1000);
  delete mLCMThread;
}

void MainWindow::loadSettings()
{
  QStringList args = QApplication::instance()->arguments();

  if (args.length() > 1)
  {
    QString filename = args[1];
    QMap<QString, QVariant> plotSettings = Json::decodeFile(filename);
    this->loadPlots(plotSettings);
  }
}

void MainWindow::loadPlots(const QMap<QString, QVariant>& plotSettings)
{
  QList<QVariant> plots = plotSettings.value("plots").toList();
  foreach (const QVariant& plot, plots)
  {
    PlotWidget* plotWidget = this->addPlot();
    plotWidget->loadSettings(plot.toMap());
  }
}

void MainWindow::onTogglePause()
{
  QPushButton* button = qobject_cast<QPushButton*>(this->sender());
  if (button->isChecked())
  {
    foreach (PlotWidget* plot, mPlots)
    {
      plot->stop();
    }
  }
  else
  {
    foreach (PlotWidget* plot, mPlots)
    {
      plot->start();
    }
  }
}

SignalHandler* MainWindow::getSignalSelectionFromUser()
{
  SelectSignalDialog dialog(this);
  int result = dialog.exec();
  if (result != QDialog::Accepted)
  {
    return 0;
  }

  return dialog.createSignalHandler();
}

void MainWindow::onNewPlotClicked()
{
  SignalHandler* signalHandler = this->getSignalSelectionFromUser();
  if (!signalHandler)
  {
    return;
  }

  PlotWidget* plot = this->addPlot();
  plot->addSignal(signalHandler);
}

PlotWidget* MainWindow::addPlot()
{
  PlotWidget* plot = new PlotWidget(mLCMThread);
  mPlotLayout->addWidget(plot);
  this->connect(plot, SIGNAL(removePlotRequested(PlotWidget*)), SLOT(onRemovePlot(PlotWidget*)));
  mPlots.append(plot);
  return plot;
}

void MainWindow::onAddSignalToPlot(PlotWidget* plot)
{
  if (!plot)
  {
    return;
  }

  SignalHandler* signalHandler = this->getSignalSelectionFromUser();
  if (!signalHandler)
  {
    return;
  }

  plot->addSignal(signalHandler);
}

void MainWindow::onRemovePlot(PlotWidget* plot)
{
  if (!plot)
  {
    return;
  }

  mPlotLayout->removeWidget(plot);
  mPlots.removeAll(plot);
  delete plot;
}
