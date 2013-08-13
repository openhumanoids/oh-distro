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
  void stop();
  void stopThreads();

  void loadSettings(const QMap<QString, QVariant>& plotSettings);
  void addSignal(const QMap<QString, QVariant>& signalSettings);
  void addSignal(SignalHandler* signalHandler);

public slots:

  void onShowContextMenu(const QPoint&);

signals:

  void addSignalRequested(PlotWidget* plot);
  void removePlotRequested(PlotWidget* plot);

private:

  Plot *d_plot;
  QList<SignalHandler*> mSignals;
  QList<QColor> mColors;
  LCMThread* mLCMThread;
};

#endif
