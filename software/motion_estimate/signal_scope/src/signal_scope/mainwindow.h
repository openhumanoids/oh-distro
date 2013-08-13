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
  void onRemovePlot(PlotWidget* plot);
  void onAddSignalToPlot(PlotWidget* plot);

protected:

  void loadSettings();

  void loadPlots(const QMap<QString, QVariant>& plotSettings);
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
