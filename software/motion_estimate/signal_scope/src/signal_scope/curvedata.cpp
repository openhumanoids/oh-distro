#include "curvedata.h"
#include "signaldata.h"


SignalData* CurveData::signalData()
{
  return mSignalData;
}

QPointF CurveData::sample(size_t i) const
{
  return mSignalData->value(i);
}

size_t CurveData::size() const
{
  return mSignalData->size();
}

QRectF CurveData::boundingRect() const
{
  return mSignalData->boundingRect();
}
