#include "mainwindow.h"
#include "plotwidget.h"
#include "lcmthread.h"

#include <qlabel.h>
#include <qlayout.h>
#include <QScrollArea>
#include <QPushButton>
#include <QShortcut>

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


  this->connect(newPlotButton, SIGNAL(clicked()), SLOT(onNewPlot()));
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

  //this->onNewPlot();

 QShortcut* quit = new QShortcut(QKeySequence(tr("Ctrl+Q", "Quit")), this);
  this->connect(quit, SIGNAL(activated()), SLOT(close()));
}

MainWindow::~MainWindow()
{
  mLCMThread->stop();
  mLCMThread->wait(1000);
  delete mLCMThread;
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

void MainWindow::onNewPlot()
{
  PlotWidget* plot = new PlotWidget(mLCMThread);
  mPlotLayout->addWidget(plot);
  this->connect(plot, SIGNAL(removePlotRequested(PlotWidget*)), SLOT(onRemovePlot(PlotWidget*)));
  plot->addSignal();
  mPlots.append(plot);
}

void MainWindow::onRemovePlot(PlotWidget* plot)
{
  if (plot)
  {
    mPlotLayout->removeWidget(plot);
    mPlots.removeAll(plot);
    delete plot;
  }
}
