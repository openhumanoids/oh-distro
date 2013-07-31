#ifndef _PLOTWIDGET_H_
#define _PLOTWIDGET_H_

#include <qwidget.h>

class Plot;
class Knob;
class WheelBox;
class SignalHandler;
class LCMThread;

class PlotWidget : public QWidget
{
  Q_OBJECT

public:
  PlotWidget(LCMThread* lcmThread, QWidget* parent=0);

  void start();
  void stopThreads();


  void addSignal();

public slots:

  void onShowContextMenu(const QPoint&);

signals:

  void removePlotRequested(PlotWidget* plot);

private:

  Plot *d_plot;
  QList<SignalHandler*> mSignals;
  QList<QColor> mColors;
  LCMThread* mLCMThread;
};

#endif
