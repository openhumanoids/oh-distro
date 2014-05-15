#ifndef _PLOTWIDGET_H_
#define _PLOTWIDGET_H_

#include <qwidget.h>
#include <qmap.h>
#include <qwt_plot_curve.h>

class Plot;
class Knob;
class WheelBox;
class SignalHandler;
class PythonChannelSubscriberCollection;
class QListWidget;
class QListWidgetItem;
class QLabel;
class QDoubleSpinBox;

class PlotWidget : public QWidget
{
  Q_OBJECT

public:
  PlotWidget(PythonChannelSubscriberCollection* subscribers, QWidget* parent=0);

  void start();
  void stop();
  void stopThreads();

  void replot();

  void setEndTime(double endTime);
  void setXAxisScale(double x0, double x1);

  void clearHistory();
  void setBackgroundColor(QString color);
  void setPointSize(double pointSize);
  void setCurveStyle(QwtPlotCurve::CurveStyle style);
  void setAlignMode(QString mode);

  void loadSettings(const QMap<QString, QVariant>& plotSettings);
  void addSignal(const QMap<QString, QVariant>& signalSettings);
  void addSignal(SignalHandler* signalHandler);

  void setSignalVisibility(SignalHandler* signalHandler, bool visible);
  bool signalIsVisible(SignalHandler* signalHandler);

  QMap<QString, QVariant> saveSettings();
  QMap<QString, QVariant> saveSignalSettings(SignalHandler* signalHandler);

  double timeWindow() const;

  QListWidgetItem* itemForSignal(SignalHandler* signalHandler);
  SignalHandler* signalForItem(QListWidgetItem* item);

  QList<SignalHandler*> signalHandlers();

public slots:

  void onShowContextMenu(const QPoint&);
  void onShowSignalContextMenu(const QPoint&);

  void onSignalListItemChanged(QListWidgetItem* item);
  void updateSignalInfoLabel();
  void onResetYAxisScale();

signals:

  void addSignalRequested(PlotWidget* plot);
  void removePlotRequested(PlotWidget* plot);
  void syncXAxisScale(double x0, double x1);

private:

  Plot *d_plot;
  QMap<QListWidgetItem*, SignalHandler*> mSignals;
  QList<QColor> mColors;
  PythonChannelSubscriberCollection* mSubscribers;

  QListWidget* mSignalListWidget;
  QLabel* mSignalInfoLabel;
  QDoubleSpinBox* mTimeWindowSpin;
};

#endif
