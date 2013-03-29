// derived from http://stackoverflow.com/questions/9802670/how-to-add-a-tick-mark-to-a-slider-if-it-cannot-inherit-qslider
#include <QtGui>
#include <algorithm>
#include <iostream>

class DefaultValueSlider : public QSlider
{
    Q_OBJECT

public:
    DefaultValueSlider(Qt::Orientation orientation, QWidget *parent = NULL)
      : QSlider(orientation, parent),
    _all_tick_positions(),
    _major_tick_positions(),
    _minor_tick_positions()
    {
        _upperbound = -1;
	_lowerbound = -1;
    }

    // tick_value is a percentage between 0.0 and 1.0
    void addMajorTick(double tick_value)
    {
        if (tick_value >= 0.0 && tick_value <= 1.0)
	{
	    _major_tick_positions.push_back(tick_value);
	    _all_tick_positions.push_back(tick_value);
	    std::sort(_major_tick_positions.begin(), _major_tick_positions.end());
	    std::sort(_all_tick_positions.begin(), _all_tick_positions.end());
	}
    }

    void addMinorTick(double tick_value)
    {
        if (tick_value >=0.0 && tick_value <= 1.0)
	{
	    _minor_tick_positions.push_back(tick_value);
	    _all_tick_positions.push_back(tick_value);
	    std::sort(_minor_tick_positions.begin(), _minor_tick_positions.end());
	    std::sort(_all_tick_positions.begin(), _all_tick_positions.end());
	}
    }

    void clearTicks()
    {
        _major_tick_positions.clear();
        _minor_tick_positions.clear();
	_all_tick_positions.clear();
    }

    // lowerbound and upperbound are percentages between 0.0 and 1.0
    void setSelectedRange(float lowerbound, float upperbound)
    {
        if (lowerbound >= 0.0 && upperbound <= 1.0 && upperbound >= lowerbound)
	{
	    _lowerbound = lowerbound;
            _upperbound = upperbound;
	}
    }

    void advanceToNextMajorTick()
    {
    }

    void returnToPreviousMajorTick()
    {
    }

    void advanceToNextTick()
    {
    }

    void returnToPreviousTick()
    {
    }

    bool hasPreviousMajorTick()
    {
        int size = (int)_major_tick_positions.size();
	return size > 0 && _major_tick_positions[0] < (float(value()) / (maximum() - minimum()));
    }

    bool hasPreviousTick()
    {
        int size = (int)_all_tick_positions.size();
        return size > 0 && _all_tick_positions[0] < (float(value()) / (maximum() - minimum()));
    }

    bool hasNextMajorTick()
    {
        int size = (int)_major_tick_positions.size();
	return size > 0 && _major_tick_positions[size - 1] > (float(value()) / (maximum() - minimum()));
    }

    bool hasNextTick()
    {
        int size = _all_tick_positions.size();
        return size > 0 && _all_tick_positions[size - 1] > (float(value()) / (maximum() - minimum()));
    }

protected:

    //input is a float from 0.0 to 1.0 representing the percentage across
    int getSliderPosition(float percent)
    {
        if (percent < 0.0)
        {
            return 0.0;
        }

        if (percent > 1.0)
        {
            return width();
        }

	return QStyle::sliderPositionFromValue(minimum(), maximum(),
					       percent * (maximum() - minimum()), width());
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

        for (int i = 0; i < (int)_major_tick_positions.size(); i++)
        {
	    int position = getSliderPosition(_major_tick_positions[i]);
            painter.drawLine(position, 0, position, height());
        }

	for (int i = 0; i < (int)_minor_tick_positions.size(); i++)
        {
	  int position = getSliderPosition(_minor_tick_positions[i]);
            painter.drawLine(position, height() / 4.0, position, height() * 3.0 / 4.0); // half height ticks
	}
	
        QSlider::paintEvent(ev);
    }

private:
    float _upperbound; // from 0.0 to 1.0
    float _lowerbound; // from 0.0 to 1.0
    std::vector<double> _all_tick_positions; 
    std::vector<double> _major_tick_positions;
    std::vector<double> _minor_tick_positions;
};
