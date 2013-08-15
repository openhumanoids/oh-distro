#include "signaldata.h"
#include <qvector.h>
#include <qmutex.h>
#include <qreadwritelock.h>


#include "fpscounter.h"


class SignalData::PrivateData
{
public:
  PrivateData()
  {
    messageError = false;
    //values.reserve(1000);
  }

  inline void append(const QPointF &sample)
  {
    values.append(sample);
    fpsCounter.update();

    // adjust the bounding rectangle
    /*
    if ( boundingRect.width() < 0 || boundingRect.height() < 0 )
    {
      boundingRect.setRect(sample.x(), sample.y(), 0.0, 0.0);
    }
    else
    {
      boundingRect.setRight(sample.x());

      if ( sample.y() > boundingRect.bottom() )
          boundingRect.setBottom(sample.y());

      if ( sample.y() < boundingRect.top() )
          boundingRect.setTop(sample.y());
    }
    */
  }

  bool messageError;

  QReadWriteLock lock;

  QVector<QPointF> values;
  QRectF boundingRect;

  QMutex mutex; // protecting pendingValues
  QVector<QPointF> pendingValues;

  FPSCounter fpsCounter;
};

SignalData::SignalData()
{
  d_data = new PrivateData();
}

SignalData::~SignalData()
{
  delete d_data;
}

int SignalData::size() const
{
  return d_data->values.size();
}

QPointF SignalData::value(int index) const
{
  return d_data->values[index];
}

QRectF SignalData::boundingRect() const
{
  return d_data->boundingRect;
}

void SignalData::lock()
{
  d_data->mutex.lock();
}

void SignalData::unlock()
{
  d_data->mutex.unlock();
}

void SignalData::append(const QPointF &sample)
{
  d_data->mutex.lock();
  d_data->pendingValues += sample;
  d_data->fpsCounter.update();
  d_data->mutex.unlock();
}

void SignalData::flagMessageError()
{
  d_data->messageError = true;
}

bool SignalData::hasMessageError() const
{
  return d_data->messageError;
}

double SignalData::messageFrequency() const
{
  d_data->mutex.lock();
  double freq = d_data->fpsCounter.averageFPS();
  d_data->mutex.unlock();
  return freq;
}

void SignalData::updateValues()
{
  d_data->mutex.lock();
  d_data->values += d_data->pendingValues;
  d_data->pendingValues.clear();
  d_data->mutex.unlock();

  if (!d_data->values.size())
  {
    return;
  }

  // All values that are older than 60 seconds will expire
  float expireTime = d_data->values.back().x() - 60;
  while (d_data->values.size() && d_data->values.front().x() < expireTime)
  {
    d_data->values.pop_front();
  }

  // recompute bounding rect
  if (d_data->values.size() > 1)
  {
    d_data->boundingRect.setLeft(d_data->values.front().x());
    d_data->boundingRect.setRight(d_data->values.back().x());

    double minY, maxY = d_data->values.front().y();
    foreach (const QPointF& point, d_data->values)
    {
      if (point.y() < minY)
        minY = point.y();

      if (point.y() > maxY)
        maxY = point.y();
    }

    d_data->boundingRect.setTop(maxY);
    d_data->boundingRect.setBottom(minY);
  }
  else
  {
    d_data->boundingRect = QRectF();
  }
}
