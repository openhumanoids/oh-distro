// derived from http://stackoverflow.com/questions/9802670/how-to-add-a-tick-mark-to-a-slider-if-it-cannot-inherit-qslider
#include <QtGui>

class DefaultValueSlider : public QSlider
{
    Q_OBJECT

public:
    DefaultValueSlider(Qt::Orientation orientation, QWidget *parent = NULL)
        : QSlider(orientation, parent),
          _tick_positions()
    {
        _upperbound = -1;
	_lowerbound = -1;
    }
    void addTick(double tick_value)
    {
        _tick_positions.push_back(tick_value);
    }
    void clearTicks()
    {
        _tick_positions.clear();
    }
    void setSelectedRange(float lowerbound, float upperbound)
    {
      if (lowerbound < 0.0)
	{
	  lowerbound = 0.0;
	}
      if (upperbound < lowerbound)
	{
	  upperbound = lowerbound;
	}
      if (upperbound > 1.0)
	{
	  upperbound = 1.0;
	}

      _lowerbound = lowerbound;
      _upperbound = upperbound;
    }

protected:

    int getSliderPosition(float num)
    {
        if (num < 0.0)
        {
            return 0.0;
        }

        if (num > 1.0)
        {
            return width();
        }

        return QStyle::sliderPositionFromValue(
                   minimum(), maximum(), num * (maximum() - minimum()), width());
    }

    void paintEvent(QPaintEvent *ev)
    {
        QPainter painter(this);

        if (_lowerbound >= 0 && _upperbound >=0)
        {
            int pos1 = getSliderPosition(_lowerbound);
            int pos2 = getSliderPosition(_upperbound);
            painter.fillRect(pos1, 0, pos2 - pos1, height(), QColor("#90EE90"));
        }

        for (int i = 0; i < (int)_tick_positions.size(); i++)
        {
	    int position = getSliderPosition(_tick_positions[i]);
            painter.drawLine(position, 0, position, height());
        }

        QSlider::paintEvent(ev);
    }

private:
    float _upperbound; // from 0.0 to 1.0
    float _lowerbound; // from 0.0 to 1.0
    std::vector<double> _tick_positions; // from 0.0 to 1.0
};
