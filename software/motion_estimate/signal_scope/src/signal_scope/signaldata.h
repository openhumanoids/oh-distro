#ifndef _SIGNAL_DATA_H_
#define _SIGNAL_DATA_H_

#include <qrect.h>

class SignalData
{
public:

    SignalData();
    virtual ~SignalData();

    void appendSample(double x, double y);
    void updateValues();

    int size() const;
    QPointF value(int index) const;

    QRectF boundingRect() const;

    void lock();
    void unlock();

    bool hasMessageError() const;
    void flagMessageError();

    void clear();

    // New signal data points per second are calculated using
    // an exponential moving average with a 1 second time window.
    double messageFrequency() const;

private:

    Q_DISABLE_COPY(SignalData);

    class PrivateData;
    PrivateData *d_data;
};

#endif
