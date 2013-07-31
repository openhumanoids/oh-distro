#include <qwt_series_data.h>

class SignalData;

class CurveData: public QwtSeriesData<QPointF>
{
public:

  CurveData(SignalData* signalData) : mSignalData(signalData) {}

  SignalData* signalData();

  virtual QPointF sample(size_t i) const;
  virtual size_t size() const;

  virtual QRectF boundingRect() const;

protected:

  SignalData* mSignalData;
};
