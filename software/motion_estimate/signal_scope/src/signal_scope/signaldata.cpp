#include "signaldata.h"
#include <qvector.h>
#include <qmutex.h>
#include <qreadwritelock.h>

class SignalData::PrivateData
{
public:
  PrivateData():
    boundingRect(1.0, 1.0, -2.0, -2.0) // invalid
  {
    values.reserve(1000);
  }

  inline void append(const QPointF &sample)
  {
    values.append(sample);

    // adjust the bounding rectangle

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
  }

  QReadWriteLock lock;

  QVector<QPointF> values;
  QRectF boundingRect;

  QMutex mutex; // protecting pendingValues
  QVector<QPointF> pendingValues;
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
  d_data->mutex.unlock();
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
}
