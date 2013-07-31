#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <qwidget.h>

class QScrollArea;
class QVBoxLayout;
class PlotWidget;
class LCMThread;

class MainWindow : public QWidget
{
    Q_OBJECT

public:

  MainWindow(QWidget * = NULL);
  ~MainWindow();

public slots:

  void onNewPlot();
  void onRemovePlot(PlotWidget* plot);

protected:


  QScrollArea* mScrollArea;
  QWidget* mPlotArea;
  QVBoxLayout* mPlotLayout;

  LCMThread* mLCMThread;

};

#endif
