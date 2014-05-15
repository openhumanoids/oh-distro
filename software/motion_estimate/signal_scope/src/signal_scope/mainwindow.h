#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <qmainwindow.h>
#include "fpscounter.h"

class QScrollArea;
class QVBoxLayout;
class QTimer;
class PlotWidget;
class LCMThread;
class PythonChannelSubscriberCollection;
class SignalHandler;

class ctkPythonConsole;
class ctkAbstractPythonManager;


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

  MainWindow(QWidget * = NULL);
  ~MainWindow();

  static QString defaultSettingsDir();

  void setPlotBackgroundColor(QString color);

public slots:

  void onTogglePause();
  void onNewPlotClicked();
  void onSaveSettings();
  void onOpenSettings();
  void onClearHistory();
  void onRemovePlot(PlotWidget* plot);
  void onAddSignalToPlot(PlotWidget* plot);
  void onRemoveAllPlots();

  void onChooseBackgroundColor();
  void onChoosePointSize();

protected slots:

  void onSyncXAxis(double x0, double x1);

  void onRedrawPlots();

  void onPointSizeChanged(int size);
  void onCurveStyleChanged(QString style);
  void onAlignModeChanged(QString mode);

protected:

  void handleCommandLineArgs();
  void saveSettings(const QString& filename);
  void loadSettings(const QString& filename);
  void loadSettings(const QMap<QString, QVariant>& settings);
  void loadPlot(const QMap<QString, QVariant>& plot);

  void testPythonSignals();
  void initPython();
  void loadPythonSignals(PlotWidget* plot, const QString& filename);

  PlotWidget* addPlot();
  QList<SignalHandler*> getSignalSelectionFromUser();

  bool mPlaying;
  QString mLastOpenFile;
  QScrollArea* mScrollArea;
  QWidget* mPlotArea;
  QVBoxLayout* mPlotLayout;
  QTimer *mRedrawTimer;

  FPSCounter mFPSCounter;

  QList<PlotWidget*> mPlots;

  LCMThread* mLCMThread;

  ctkPythonConsole* mConsole;
  ctkAbstractPythonManager* mPythonManager;
  PythonChannelSubscriberCollection* mSubscribers;

  class Internal;
  Internal* mInternal;

};

#endif
