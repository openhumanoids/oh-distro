#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <qwidget.h>

class QScrollArea;
class QVBoxLayout;
class PlotWidget;
class LCMThread;
class SignalHandler;

class MainWindow : public QWidget
{
    Q_OBJECT

public:

  MainWindow(QWidget * = NULL);
  ~MainWindow();

public slots:

  void onTogglePause();
  void onNewPlotClicked();
  void onSaveSettings();
  void onRemovePlot(PlotWidget* plot);
  void onAddSignalToPlot(PlotWidget* plot);

protected:

  void handleCommandLineArgs();
  void saveSettings(const QString& filename);
  void loadSettings(const QString& filename);
  void loadSettings(const QMap<QString, QVariant>& settings);
  void loadPlot(const QMap<QString, QVariant>& plot);

  PlotWidget* addPlot();
  SignalHandler* getSignalSelectionFromUser();

  QScrollArea* mScrollArea;
  QWidget* mPlotArea;
  QVBoxLayout* mPlotLayout;

  QList<PlotWidget*> mPlots;

  LCMThread* mLCMThread;

};

#endif
