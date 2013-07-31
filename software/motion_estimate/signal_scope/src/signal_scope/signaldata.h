#ifndef _SIGNAL_DATA_H_
#define _SIGNAL_DATA_H_ 1

#include <qrect.h>

class SignalData
{
public:

    SignalData();
    virtual ~SignalData();

    void append(const QPointF &pos);
    void updateValues();

    int size() const;
    QPointF value(int index) const;

    QRectF boundingRect() const;

    void lock();
    void unlock();

private:


    Q_DISABLE_COPY(SignalData);



    class PrivateData;
    PrivateData *d_data;
};

#endif
