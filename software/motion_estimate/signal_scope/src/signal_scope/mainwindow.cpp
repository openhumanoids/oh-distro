#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "plotwidget.h"
#include "signalhandler.h"
#include "signaldata.h"
#include "selectsignaldialog.h"
#include "signaldescription.h"
#include "lcmthread.h"

#include <QLabel>
#include <QLayout>
#include <QApplication>
#include <QDebug>
#include <QScrollArea>
#include <QPushButton>
#include <QShortcut>
#include <QFileDialog>
#include <QInputDialog>
#include <QTimer>

#include "qjson.h"

#include "ctkPythonConsole.h"
#include "ctkAbstractPythonManager.h"
#include "pythonsignalhandler.h"
#include "pythonchannelsubscribercollection.h"


#include <cstdio>
#include <limits>


class MainWindow::Internal : public Ui::MainWindow
{
public:

};


MainWindow::MainWindow(QWidget* parent): QMainWindow(parent)
{
  mInternal = new Internal;
  mInternal->setupUi(this);

  mPlaying = false;
  this->setWindowTitle("Signal Scope");

  mLCMThread = new LCMThread;
  mLCMThread->start();

  this->initPython();


  mScrollArea = new QScrollArea;
  mPlotArea = new QWidget;
  mPlotLayout = new QVBoxLayout(mPlotArea);

  mScrollArea->setWidget(mPlotArea);
  mScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  mScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  mScrollArea->setWidgetResizable(true);
  this->setCentralWidget(mScrollArea);

  mInternal->ActionOpen->setIcon(qApp->style()->standardIcon(QStyle::SP_DialogOpenButton));
  mInternal->ActionSave->setIcon(qApp->style()->standardIcon(QStyle::SP_DialogSaveButton));
  mInternal->ActionPause->setIcon(qApp->style()->standardIcon(QStyle::SP_MediaPlay));
  mInternal->ActionClearHistory->setIcon(qApp->style()->standardIcon(QStyle::SP_TrashIcon));
  mInternal->ActionAddPlot->setIcon(qApp->style()->standardIcon(QStyle::SP_TrashIcon));
  //QStyle::SP_DialogDiscardButton

  this->connect(mInternal->ActionQuit, SIGNAL(triggered()), SLOT(close()));
  this->connect(mInternal->ActionOpen, SIGNAL(triggered()), SLOT(onOpenSettings()));
  this->connect(mInternal->ActionSave, SIGNAL(triggered()), SLOT(onSaveSettings()));
  this->connect(mInternal->ActionPause, SIGNAL(triggered()), SLOT(onTogglePause()));
  this->connect(mInternal->ActionAddPlot, SIGNAL(triggered()), SLOT(onNewPlotClicked()));
  this->connect(mInternal->ActionClearHistory, SIGNAL(triggered()), SLOT(onClearHistory()));

  this->connect(mInternal->ActionBackgroundColor, SIGNAL(triggered()), SLOT(onChooseBackgroundColor()));

  mRedrawTimer = new QTimer(this);
  //mRedrawTimer->setSingleShot(true);
  this->connect(mRedrawTimer, SIGNAL(timeout()), this, SLOT(onRedrawPlots()));

  QShortcut* showConsole = new QShortcut(QKeySequence("F8"), this);
  this->connect(showConsole, SIGNAL(activated()), this->mConsole, SLOT(show()));

  this->connect(new QShortcut(QKeySequence("Ctrl+W"), this->mConsole), SIGNAL(activated()), this->mConsole, SLOT(close()));

  QString closeShortcut = "Ctrl+D";
  #ifdef Q_OS_DARWIN
  closeShortcut = "Meta+D";
  #endif
  this->connect(new QShortcut(QKeySequence(closeShortcut), this->mConsole), SIGNAL(activated()), this->mConsole, SLOT(close()));

  this->resize(1024,800);
  this->handleCommandLineArgs();

  this->onTogglePause();

  //this->testPythonSignals();
}

MainWindow::~MainWindow()
{

  QString settingsFile = QDir::homePath() + "/.signal_scope.json";
  this->saveSettings(settingsFile);

  mLCMThread->stop();
  mLCMThread->wait(250);
  delete mLCMThread;

  delete mInternal;
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

void MainWindow::testPythonSignals()
{
  this->onRemoveAllPlots();
  PlotWidget* plot = this->addPlot();

  QString testFile = QString(getenv("DRC_BASE")) + "/software/motion_estimate/signal_scope/src/signal_scope/userSignals.py";
  this->loadPythonSignals(plot, testFile);
}

void MainWindow::initPython()
{
  this->mPythonManager = new ctkAbstractPythonManager(this);
  this->mConsole = new ctkPythonConsole(this);
  this->mConsole->setWindowFlags(Qt::Dialog);
  this->mConsole->initialize(this->mPythonManager);
  this->mConsole->setAttribute(Qt::WA_QuitOnClose, true);
  this->mConsole->resize(600, 280);
  this->mConsole->setProperty("isInteractive", true);
  this->mPythonManager->addObjectToPythonMain("_console", this->mConsole);

  this->mPythonManager->executeFile(QString(getenv("DRC_BASE")) + "/software/motion_estimate/signal_scope/src/signal_scope/signalScopeSetup.py");
  PythonQtObjectPtr mainContext = PythonQt::self()->getMainModule();
  PythonQtObjectPtr decodeCallback = PythonQt::self()->getVariable(mainContext, "decodeMessageFunction");

  this->mSubscribers = new PythonChannelSubscriberCollection(mLCMThread, decodeCallback, this);
}

void MainWindow::loadPythonSignals(PlotWidget* plot, const QString& filename)
{
  this->mPythonManager->executeFile(filename);
  PythonQtObjectPtr mainContext = PythonQt::self()->getMainModule();
  QList<QVariant> signalsMap = PythonQt::self()->getVariable(mainContext, "signals").toList();
  foreach (const QVariant& signalItem, signalsMap)
  {
    QList<QVariant> signalItemList = signalItem.toList();
    QString channel = signalItemList[0].toString();
    PythonQtObjectPtr callback = signalItemList[1].value<PythonQtObjectPtr>();

    SignalDescription signalDescription;
    signalDescription.mChannel = channel;
    PythonSignalHandler* signalHandler = new PythonSignalHandler(&signalDescription, callback);
    plot->addSignal(signalHandler);
  }
}

void MainWindow::onClearHistory()
{
  foreach (PlotWidget* plot, mPlots)
  {
    plot->clearHistory();
  }
}

QString MainWindow::defaultSettingsDir()
{
  QString configDir = qgetenv("DRC_BASE") + "/software/config/signal_scope_configs";
  if (QDir(configDir).exists())
  {
    return QDir(configDir).canonicalPath();
  }
  else
  {
    return QDir::currentPath();
  }
}

void MainWindow::onChooseBackgroundColor()
{
  QStringList colors;
  colors << "Black" << "White";

  bool ok;
  QString color = QInputDialog::getItem(this, "Choose background color", "Color", colors, 0, false, &ok);
  if (ok)
  {
    this->setPlotBackgroundColor(color);
  }
}

void MainWindow::onChoosePointSize()
{
  bool ok;
  int pointSize = QInputDialog::getInt(this, "Choose point size", "Point size", 1, 1, 20, 1, &ok);
  if (ok)
  {
    foreach (PlotWidget* plot, mPlots)
    {
      plot->setPointSize(pointSize - 1);
    }
  }
}

void MainWindow::setPlotBackgroundColor(QString color)
{
  foreach (PlotWidget* plot, mPlots)
  {
    plot->setBackgroundColor(color);
  }
}

void MainWindow::onSyncXAxis(double x0, double x1)
{
  foreach (PlotWidget* plot, mPlots)
  {
    if (plot == this->sender())
      continue;

    plot->setXAxisScale(x0, x1);
    plot->replot();
  }
}

void MainWindow::onRedrawPlots()
{
  mFPSCounter.update();
  //printf("redraw fps: %f\n", this->mFPSCounter.averageFPS());

  if (mPlots.isEmpty())
  {
    return;
  }


  QList<SignalData*> signalDataList;
  foreach (PlotWidget* plot, mPlots)
  {
    foreach (SignalHandler* signalHandler, plot->signalHandlers())
    {
      signalDataList.append(signalHandler->signalData());
    }
  }

  if (signalDataList.isEmpty())
  {
    return;
  }

  float maxTime = -std::numeric_limits<float>::max();

  foreach (SignalData* signalData, signalDataList)
  {
    signalData->updateValues();
    if (signalData->size())
    {
      float signalMaxTime = signalData->boundingRect().right();
      if (signalMaxTime > maxTime)
      {
        maxTime = signalMaxTime;
      }
    }
  }

  if (maxTime == -std::numeric_limits<float>::max())
  {
    return;
  }

  foreach (PlotWidget* plot, mPlots)
  {
    plot->setEndTime(maxTime);
    plot->replot();
  }
}

void MainWindow::onOpenSettings()
{
  QString filter = "JSON (*.json)";
  QString filename = QFileDialog::getOpenFileName(this, "Open Settings", this->defaultSettingsDir(), filter);
  if (filename.length())
  {
    this->onRemoveAllPlots();
    this->loadSettings(filename);
  }
}

void MainWindow::onSaveSettings()
{
  QString defaultFile = mLastOpenFile.isEmpty() ? this->defaultSettingsDir() : mLastOpenFile;
  QString filter = "JSON (*.json)";
  QString filename = QFileDialog::getSaveFileName(this, "Save Settings", defaultFile, filter);
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
    QMap<QString, QVariant> plotSettings = plot.toMap();
    plotWidget->loadSettings(plotSettings);

    QString pythonFile = plotSettings.value("pythonScript").toString();
    if (pythonFile.length())
    {
      pythonFile = QString(getenv("DRC_BASE")) + "/" + pythonFile;
      this->loadPythonSignals(plotWidget, pythonFile);
    }
  }

  int windowWidth = settings.value("windowWidth", 1024).toInt();
  int windowHeight = settings.value("windowHeight", 800).toInt();
  this->resize(windowWidth, windowHeight);
}

void MainWindow::onTogglePause()
{
  mPlaying = !mPlaying;
  mInternal->ActionPause->setChecked(mPlaying);
  mInternal->ActionPause->setIcon(qApp->style()->standardIcon(mPlaying ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay));
  mInternal->ActionPause->setText(mPlaying ? "Pause" : "Play");


  foreach (PlotWidget* plot, mPlots)
  {
    if (mPlaying)
      plot->start();
    else
      plot->stop();
  }

  if (mPlaying)
  {
    mRedrawTimer->start(33);
  }
  else
  {
    mRedrawTimer->stop();
  }
}

QList<SignalHandler*> MainWindow::getSignalSelectionFromUser()
{
  SelectSignalDialog dialog(this);
  int result = dialog.exec();
  if (result != QDialog::Accepted)
  {
    return QList<SignalHandler*>();
  }

  return dialog.createSignalHandlers();
}

void MainWindow::onNewPlotClicked()
{
  QList<SignalHandler*> signalHandlers = this->getSignalSelectionFromUser();
  if (signalHandlers.isEmpty())
  {
    return;
  }

  PlotWidget* plot = this->addPlot();
  foreach (SignalHandler* signalHandler, signalHandlers)
  {
    plot->addSignal(signalHandler);
  }
}

PlotWidget* MainWindow::addPlot()
{
  PlotWidget* plot = new PlotWidget(mSubscribers);
  mPlotLayout->addWidget(plot);
  this->connect(plot, SIGNAL(removePlotRequested(PlotWidget*)), SLOT(onRemovePlot(PlotWidget*)));
  this->connect(plot, SIGNAL(addSignalRequested(PlotWidget*)), SLOT(onAddSignalToPlot(PlotWidget*)));
  this->connect(plot, SIGNAL(syncXAxisScale(double, double)), SLOT(onSyncXAxis(double, double)));
  mPlots.append(plot);

  if (mPlaying)
    plot->start();
  else
    plot->stop();

  return plot;
}

void MainWindow::onAddSignalToPlot(PlotWidget* plot)
{
  if (!plot)
  {
    return;
  }

  QList<SignalHandler*> signalHandlers = this->getSignalSelectionFromUser();
  if (signalHandlers.isEmpty())
  {
    return;
  }

  foreach (SignalHandler* signalHandler, signalHandlers)
  {
    plot->addSignal(signalHandler);
  }
}

void MainWindow::onRemoveAllPlots()
{
  QList<PlotWidget*> plots = mPlots;
  foreach(PlotWidget* plot, plots)
  {
    this->onRemovePlot(plot);
  }
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
