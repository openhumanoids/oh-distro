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
#include <QFileDialog>

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

  QPushButton* saveSettingsButton = new QPushButton("Save settings");
  hlayout->addWidget(saveSettingsButton);

  hlayout->addStretch();


  this->connect(newPlotButton, SIGNAL(clicked()), SLOT(onNewPlotClicked()));
  this->connect(pauseButton, SIGNAL(clicked()), SLOT(onTogglePause()));
  this->connect(saveSettingsButton, SIGNAL(clicked()), SLOT(onSaveSettings()));

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


  this->resize(1024,800);
  this->handleCommandLineArgs();
}

MainWindow::~MainWindow()
{

  QString settingsFile = QDir::homePath() + "/.signal_scope.json";
  this->saveSettings(settingsFile);

  mLCMThread->stop();
  mLCMThread->wait(250);
  delete mLCMThread;
}

void MainWindow::handleCommandLineArgs()
{
  QStringList args = QApplication::instance()->arguments();

  if (args.length() > 1)
  {
    QString filename = args[1];
    this->loadSettings(filename);
  }
  else
  {
    QString settingsFile = QDir::homePath() + "/.signal_scope.json";
    if (QFileInfo(settingsFile).exists())
    {
      this->loadSettings(settingsFile);
    }
  }
}

void MainWindow::onSaveSettings()
{
  QString filename = QFileDialog::getSaveFileName(this, "Save Settings", ".json");
  if (filename.length())
  {
    this->saveSettings(filename);
  }
}

void MainWindow::saveSettings(const QString& filename)
{
  QMap<QString, QVariant> settings;

  settings["windowWidth"] = this->width();
  settings["windowHeight"] = this->height();

  QList<QVariant> plotSettings;
  foreach (PlotWidget* plot, mPlots)
  {
    plotSettings.append(plot->saveSettings());
  }

  settings["plots"] = plotSettings;

  Json::encodeFile(filename, settings);
}

void MainWindow::loadSettings(const QString& filename)
{
  QMap<QString, QVariant> settings = Json::decodeFile(filename);
  this->loadSettings(settings);
}

void MainWindow::loadSettings(const QMap<QString, QVariant>& settings)
{
  QList<QVariant> plots = settings.value("plots").toList();
  foreach (const QVariant& plot, plots)
  {
    PlotWidget* plotWidget = this->addPlot();
    plotWidget->loadSettings(plot.toMap());
  }

  int windowWidth = settings.value("windowWidth", 1024).toInt();
  int windowHeight = settings.value("windowHeight", 800).toInt();
  this->resize(windowWidth, windowHeight);
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
  this->connect(plot, SIGNAL(addSignalRequested(PlotWidget*)), SLOT(onAddSignalToPlot(PlotWidget*)));
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
