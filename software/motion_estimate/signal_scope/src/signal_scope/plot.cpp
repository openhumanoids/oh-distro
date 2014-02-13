#include "plot.h"
#include "curvedata.h"
#include "signaldata.h"
#include <qwt_plot_grid.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_directpainter.h>
#include <qwt_curve_fitter.h>
#include <qwt_painter.h>
#include <qwt_scale_engine.h>
#include <qwt_scale_draw.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_magnifier.h>
#include <qwt_text.h>

#include <qevent.h>

#include <cassert>

#define DEFAULT_CURVE_STYLE QwtPlotCurve::Dots

class MyPanner : public QObject
{
public:

  QPoint mInitialPos;
  bool mEnabled;
  int mMouseButton;
  int mKeyboardButton;
  Plot* mPlot;

  MyPanner(Plot* plot) : QObject(plot)
  {
    mEnabled = false;
    mMouseButton = Qt::LeftButton;
    mKeyboardButton = Qt::NoButton;
    mPlot = plot;
    mPlot->canvas()->installEventFilter(this);
  }

  bool eventFilter( QObject *object, QEvent *event )
  {
    switch ( event->type() )
    {
      case QEvent::MouseButtonPress:
      {
        widgetMousePressEvent( ( QMouseEvent * )event );
        break;
      }
      case QEvent::MouseMove:
      {
        widgetMouseMoveEvent( ( QMouseEvent * )event );
        break;
      }
      case QEvent::MouseButtonRelease:
      {
        widgetMouseReleaseEvent( ( QMouseEvent * )event );
        break;
      }
    }

    return false;
  }

  void moveCanvas( int dx, int dy )
  {
    if ( dx == 0 && dy == 0 )
        return;

    if (!mPlot->isStopped())
      dx = 0;

    for ( int axis = 0; axis < QwtPlot::axisCnt; axis++ )
    {
      const QwtScaleMap map = mPlot->canvasMap( axis );
      const double p1 = map.transform( mPlot->axisScaleDiv( axis )->lowerBound() );
      const double p2 = map.transform( mPlot->axisScaleDiv( axis )->upperBound() );

      double d1, d2;
      if ( axis == QwtPlot::xBottom || axis == QwtPlot::xTop )
      {
        d1 = map.invTransform( p1 - dx );
        d2 = map.invTransform( p2 - dx );
      }
      else
      {
        d1 = map.invTransform( p1 - dy );
        d2 = map.invTransform( p2 - dy );
      }
      mPlot->setAxisScale( axis, d1, d2 );
    }

    mPlot->flagAxisSyncRequired();
    mPlot->replot();
  }


  void widgetMousePressEvent( QMouseEvent *mouseEvent )
  {
    if (mouseEvent->button() != mMouseButton)
    {
      return;
    }

    if ((mouseEvent->modifiers() & Qt::KeyboardModifierMask) != (mKeyboardButton & Qt::KeyboardModifierMask))
    {
      return;
    }


    mInitialPos = mouseEvent->pos();
    mEnabled = true;
  }

  void widgetMouseMoveEvent( QMouseEvent *mouseEvent )
  {
    if (!mEnabled)
      return;

    QPoint pos = mouseEvent->pos();
    if (pos != mInitialPos)
    {
      moveCanvas(pos.x() - mInitialPos.x(), pos.y() - mInitialPos.y());
      mInitialPos = mouseEvent->pos();
    }
  }

  void widgetMouseReleaseEvent( QMouseEvent *mouseEvent )
  {
    mEnabled = false;
  }

};

class MyScaleDraw : public QwtScaleDraw
{
  virtual QwtText label(double value) const
  {
    return QString::number(value, 'f', 2);
  }
};

class MyZoomer: public QwtPlotZoomer
{
public:

    Plot* mPlot;

    MyZoomer(Plot *plot) : QwtPlotZoomer(plot->canvas())
    {
      mPlot = plot;
      setTrackerMode(AlwaysOn);
    }

    virtual QwtText trackerTextF(const QPointF &pos) const
    {
      QColor bg(Qt::white);
      bg.setAlpha(200);

      QwtText text = QwtPlotZoomer::trackerTextF(pos);
      text.setBackgroundBrush( QBrush( bg ));
      return text;
    }

protected:

    virtual void rescale()
    {
      mPlot->flagAxisSyncRequired();
      QwtPlotZoomer::rescale();
    }

};

class MyMagnifier: public QwtPlotMagnifier
{
public:
    Plot* mPlot;

    MyMagnifier(Plot* plot) : QwtPlotMagnifier(plot->canvas())
    {
      mPlot = plot;
    }

protected:

    // Normally, a value < 1.0 zooms in, a value > 1.0 zooms out.
    // This function is overloaded to invert the magnification direction.
    virtual void rescale( double factor )
    {
      factor = qAbs( factor );
      factor = (1-factor) + 1;

      mPlot->flagAxisSyncRequired();
      this->QwtPlotMagnifier::rescale(factor);
    }

};

Plot::Plot(QWidget *parent):
  QwtPlot(parent),
  d_origin(0),
  d_grid(0),
  mStopped(true),
  mAxisSyncRequired(false),
  mColorMode(0),
  mTimeWindow(10.0)
{
  setAutoReplot(false);

  // The backing store is important, when working with widget
  // overlays ( f.e rubberbands for zooming ). 
  // Here we don't have them and the internal 
  // backing store of QWidget is good enough.

  canvas()->setPaintAttribute(QwtPlotCanvas::BackingStore, false);


#if defined(Q_WS_X11)
  // Even if not recommended by TrollTech, Qt::WA_PaintOutsidePaintEvent
  // works on X11. This has a nice effect on the performance.

  canvas()->setAttribute(Qt::WA_PaintOutsidePaintEvent, true);

  // Disabling the backing store of Qt improves the performance
  // for the direct painter even more, but the canvas becomes
  // a native window of the window system, receiving paint events
  // for resize and expose operations. Those might be expensive
  // when there are many points and the backing store of
  // the canvas is disabled. So in this application
  // we better don't both backing stores.

  if ( canvas()->testPaintAttribute( QwtPlotCanvas::BackingStore ) )
  {
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
    canvas()->setAttribute(Qt::WA_NoSystemBackground, true);
  }

#endif



  plotLayout()->setAlignCanvasToScales(true);

  setAxisAutoScale(QwtPlot::xBottom, false);
  setAxisAutoScale(QwtPlot::yLeft, false);

  setAxisTitle(QwtPlot::xBottom, "Time [s]");
  setAxisScale(QwtPlot::xBottom, 0, 10);
  setAxisScale(QwtPlot::yLeft, -4.0, 4.0);

  setAxisScaleDraw(QwtPlot::xBottom, new MyScaleDraw);

  initBackground();

  QwtPlotZoomer* zoomer = new MyZoomer(this);
  zoomer->setMousePattern(QwtEventPattern::QwtEventPattern::MouseSelect1,
      Qt::LeftButton, Qt::ShiftModifier);
  // disable MouseSelect3 action of the zoomer
  zoomer->setMousePattern(QwtEventPattern::MouseSelect3, 0);

  MyPanner *panner = new MyPanner(this);

  // zoom in/out with the wheel
  mMagnifier = new MyMagnifier(this);
  mMagnifier->setMouseButton(Qt::MiddleButton);

  const QColor c(Qt::darkBlue);
  zoomer->setRubberBandPen(c);
  zoomer->setTrackerPen(c);

  this->setMinimumHeight(200);
}

Plot::~Plot()
{

}

void Plot::addSignal(SignalData* signalData, QColor color)
{
  QwtPlotCurve* d_curve = new QwtPlotCurve();
  d_curve->setStyle(DEFAULT_CURVE_STYLE);

  QPen curvePen(color);
  curvePen.setWidth(0);

  d_curve->setPen(curvePen);

#if 1
  d_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
#endif
#if 1
  d_curve->setPaintAttribute(QwtPlotCurve::ClipPolygons, false);
#endif
  d_curve->setData(new CurveData(signalData));
  d_curve->attach(this);

  mSignals[signalData] = d_curve;
}

void Plot::removeSignal(SignalData* signalData)
{
  if (!signalData)
  {
    return;
  }

  QwtPlotCurve* curve = mSignals.value(signalData);
  assert(curve);
  curve->detach();
  delete curve;
  mSignals.remove(signalData);
}

void Plot::setSignalVisible(SignalData* signalData, bool visible)
{
  if (!signalData)
  {
    return;
  }

  QwtPlotCurve* curve = mSignals.value(signalData);
  assert(curve);

  if (visible)
  {
    curve->attach(this);
  }
  else
  {
    curve->detach();
  }

  this->replot();
}

void Plot::setSignalColor(SignalData* signalData, QColor color)
{
  if (!signalData)
  {
    return;
  }

  QwtPlotCurve* curve = mSignals.value(signalData);
  assert(curve);
  curve->setPen(QPen(color));
}

void Plot::setPointSize(double pointSize)
{
  foreach (QwtPlotCurve* curve, mSignals.values())
  {
    QPen curvePen = curve->pen();
    curvePen.setWidth(pointSize);
    curve->setPen(curvePen);
  }
  this->replot();
}

void Plot::setCurveStyle(QwtPlotCurve::CurveStyle style)
{
  foreach (QwtPlotCurve* curve, mSignals.values())
  {
    curve->setStyle(style);
  }
  this->replot();
}

void Plot::setBackgroundColor(QString color)
{
  if (color == "White")
  {
    mColorMode = 0;
  }
  else if (color == "Black")
  {
    mColorMode = 1;
  }

  this->initBackground();
}

void Plot::initBackground()
{
  if (!d_grid)
  {
    d_grid = new QwtPlotGrid();
    d_grid->enableX(false);
    d_grid->enableXMin(false);
    d_grid->enableY(true);
    d_grid->enableYMin(false);
    d_grid->attach(this);
  }

  if (!d_origin)
  {
    d_origin = new QwtPlotMarker();
    d_origin->setLineStyle(QwtPlotMarker::HLine);
    d_origin->setValue(0.0, 0.0);
    d_origin->attach(this);
  }

  QColor backgroundColor = Qt::white;
  QColor gridColor = Qt::gray;
  if (mColorMode == 1)
  {
    backgroundColor = Qt::black;
    gridColor = QColor(100, 100, 100);
  }

  QPalette pal = canvas()->palette();
  pal.setBrush(QPalette::Window, QBrush(backgroundColor));
  canvas()->setPalette(pal);

  d_grid->setPen(QPen(gridColor, 0.0, Qt::DotLine));
  d_origin->setLinePen(QPen(gridColor, 0.0, Qt::DashLine));
}

void Plot::start()
{
  mStopped = false;
  mMagnifier->setAxisEnabled(QwtPlot::xBottom, mStopped);
}

void Plot::stop()
{
  mStopped = true;
  mMagnifier->setAxisEnabled(QwtPlot::xBottom, mStopped);
}

bool Plot::isStopped()
{
  return mStopped;
}

void Plot::replot()
{
  this->updateTicks();

  // Lock the signal data objects, then plot the data and unlock.
  QList<SignalData*> signalDataList = mSignals.keys();
  foreach (SignalData* signalData, signalDataList)
  {
    signalData->lock();
  }

  QwtPlot::replot();

  foreach (SignalData* signalData, signalDataList)
  {
    signalData->unlock();
  }
}

void Plot::flagAxisSyncRequired()
{
  mAxisSyncRequired = true;
}


double Plot::timeWindow()
{
  return mTimeWindow;
}


void Plot::setTimeWindow(double interval)
{
  if ( interval > 0.0 && interval != mTimeWindow)
  {
    mTimeWindow = interval;
    QwtInterval xinterval = this->axisInterval(QwtPlot::xBottom);
    xinterval.setMinValue(xinterval.maxValue() - interval);
    this->setAxisScale(QwtPlot::xBottom, xinterval.minValue(), xinterval.maxValue());
    this->replot();
  }
}


void Plot::setYScale(double scale)
{
  setAxisScale(QwtPlot::yLeft, -scale, scale);
  this->replot();
}


void Plot::setEndTime(double endTime)
{
  QwtInterval xinterval = this->axisInterval(QwtPlot::xBottom);
  if (xinterval.maxValue() == endTime)
  {
    return;
  }

  xinterval = QwtInterval(endTime - xinterval.width(), endTime);
  this->setAxisScale(QwtPlot::xBottom, xinterval.minValue(), xinterval.maxValue());
}

void Plot::updateTicks()
{
  this->updateAxes();
  QwtInterval xinterval = this->axisInterval(QwtPlot::xBottom);

  // when playing, always use the requested time window
  // when paused, the user may zoom in/out, changing the time window
  if (!this->isStopped())
  {
    xinterval.setMinValue(xinterval.maxValue() - mTimeWindow);
  }

  QwtScaleEngine* engine = axisScaleEngine(QwtPlot::xBottom);
  QwtScaleDiv scaleDiv = engine->divideScale(xinterval.minValue(), xinterval.maxValue(), 0, 0);

  QList<double> majorTicks;
  double majorStep = scaleDiv.range() / 5.0;
  for (int i = 0; i <= 5; ++i)
  {
    majorTicks << scaleDiv.lowerBound() + i*majorStep;
  }
  majorTicks.back() = scaleDiv.upperBound();


  QList<double> minorTicks;
  double minorStep = scaleDiv.range() / 25.0;
  for (int i = 0; i <= 25; ++i)
  {
    minorTicks << scaleDiv.lowerBound() + i*minorStep;
  }
  minorTicks.back() = scaleDiv.upperBound();


  scaleDiv.setTicks(QwtScaleDiv::MajorTick, majorTicks);
  scaleDiv.setTicks(QwtScaleDiv::MinorTick, minorTicks);
  setAxisScaleDiv(QwtPlot::xBottom, scaleDiv);

  if (mAxisSyncRequired)
  {
    emit this->syncXAxisScale(xinterval.minValue(), xinterval.maxValue());
    mAxisSyncRequired = false;
  }
}
