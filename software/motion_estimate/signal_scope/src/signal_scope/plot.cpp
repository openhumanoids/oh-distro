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

#include <limits>
#include <cassert>


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
    MyZoomer(QwtPlotCanvas *canvas):
        QwtPlotZoomer(canvas)
    {
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
};

class MyMagnifier: public QwtPlotMagnifier
{
public:
    MyMagnifier(QwtPlotCanvas *canvas):
        QwtPlotMagnifier(canvas)
    {

    }

protected:

    // Normally, a value < 1.0 zooms in, a value > 1.0 zooms out.
    // This function is overloaded to invert the magnification direction.
    virtual void rescale( double factor )
    {
      factor = qAbs( factor );
      factor = (1-factor) + 1;
      this->QwtPlotMagnifier::rescale(factor);
    }

};

Plot::Plot(QWidget *parent):
  QwtPlot(parent),
  d_interval(0.0, 10.0),
  d_timerId(-1)
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

  initGradient();

  plotLayout()->setAlignCanvasToScales(true);

  setAxisTitle(QwtPlot::xBottom, "Time [s]");
  setAxisScale(QwtPlot::xBottom, d_interval.minValue(), d_interval.maxValue());
  setAxisScale(QwtPlot::yLeft, -4.0, 4.0);

  setAxisScaleDraw(QwtPlot::xBottom, new MyScaleDraw);


  QwtPlotGrid *grid = new QwtPlotGrid();
  grid->setPen(QPen(Qt::gray, 0.0, Qt::DotLine));
  grid->enableX(false);
  grid->enableXMin(false);
  grid->enableY(true);
  grid->enableYMin(false);
  grid->attach(this);

  d_origin = new QwtPlotMarker();
  d_origin->setLineStyle(QwtPlotMarker::HLine);
  d_origin->setValue(d_interval.minValue() + d_interval.width() / 2.0, 0.0);
  d_origin->setLinePen(QPen(Qt::gray, 0.0, Qt::DashLine));
  d_origin->attach(this);


  QwtPlotZoomer* zoomer = new MyZoomer(canvas());
    zoomer->setMousePattern(QwtEventPattern::QwtEventPattern::MouseSelect1,
        Qt::LeftButton, Qt::ShiftModifier);
   // zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
   //     Qt::RightButton);

    QwtPlotPanner *panner = new QwtPlotPanner(canvas());
    //panner->setAxisEnabled(QwtPlot::yRight, false);
    panner->setMouseButton(Qt::LeftButton);


    // zoom in/out with the wheel
    QwtPlotMagnifier* magnifier = new MyMagnifier(canvas());
    magnifier->setMouseButton(Qt::MiddleButton);

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
  d_curve->setStyle(QwtPlotCurve::Lines);
  d_curve->setPen(QPen(color));
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

void Plot::initGradient()
{
  QPalette pal = canvas()->palette();

#if QT_VERSION >= 0x040400
  QLinearGradient gradient( 0.0, 0.0, 1.0, 0.0 );
  gradient.setCoordinateMode( QGradient::StretchToDeviceMode );
  //gradient.setColorAt(0.0, QColor( 0, 49, 110 ) );
  //gradient.setColorAt(1.0, QColor( 0, 87, 174 ) );

  gradient.setColorAt(0.0, QColor( 255, 255, 255 ) );
  gradient.setColorAt(1.0, QColor( 255, 255, 255 ) );

  pal.setBrush(QPalette::Window, QBrush(gradient));
#else
  pal.setBrush(QPalette::Window, QBrush( color ));
#endif

  canvas()->setPalette(pal);
}

void Plot::start()
{
  d_timerId = startTimer(33);
}

void Plot::stop()
{
  killTimer(d_timerId);
}

void Plot::replot()
{
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


double Plot::timeWindow()
{
  return d_interval.width();
}


void Plot::setTimeWindow(double interval)
{
  if ( interval > 0.0 && interval != d_interval.width() )
  {
    d_interval.setMinValue(d_interval.maxValue() - interval);
  }
}


void Plot::setYScale(double scale)
{
  setAxisScale(QwtPlot::yLeft, -scale, scale);
}


void Plot::timerEvent(QTimerEvent *event)
{
  if ( event->timerId() == d_timerId )
  {
    this->fpsCounter.update();
    //printf("plot fps: %f\n", this->fpsCounter.averageFPS());

    if (!mSignals.size())
    {
      return;
    }

    float maxTime = std::numeric_limits<float>::min();

    QList<SignalData*> signalDataList = mSignals.keys();
    foreach (SignalData* signalData, signalDataList)
    {
      signalData->updateValues();
      if (signalData->size())
      {
        float signalMaxTime = signalData->value(signalData->size() - 1).x();
        if (signalMaxTime > maxTime)
          {
          maxTime = signalMaxTime;
          }
      }
    }

    if (maxTime != std::numeric_limits<float>::min())
    {
      d_interval = QwtInterval(maxTime - d_interval.width(), maxTime);

      setAxisScale(QwtPlot::xBottom, d_interval.minValue(), d_interval.maxValue());


      QwtScaleEngine* engine = axisScaleEngine(QwtPlot::xBottom);
      //engine->setAttribute(QwtScaleEngine::Floating, true);
      //engine->setMargins(0,50.0);

      QwtScaleDiv scaleDiv = engine->divideScale(d_interval.minValue(), d_interval.maxValue(), 0, 0);

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

/*
      QwtScaleDiv scaleDiv;// = *axisScaleDiv(QwtPlot::xBottom);
      scaleDiv.setInterval(d_interval);

      QList<double> majorTicks;
      majorTicks << d_interval.minValue() << d_interval.maxValue();
      scaleDiv.setTicks(QwtScaleDiv::MajorTick, majorTicks);
      setAxisScaleDiv(QwtPlot::xBottom, scaleDiv);
*/

      //printf("update x axis interval: [%f,  %f]\n", d_interval.minValue(), d_interval.maxValue());
    }






    this->replot();
  }

  QwtPlot::timerEvent(event);
}
