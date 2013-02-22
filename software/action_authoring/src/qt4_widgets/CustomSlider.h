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
        _selected_range = -1;
        //    connect(this, SIGNAL(valueChanged(int)), SLOT(VerifyDefaultValue(int)));
    }
    void addTick(double tick_value)
    {
        _tick_positions.push_back(tick_value);
    }
    void clearTicks()
    {
        _tick_positions.clear();
    }
    void setSelectedRangeIndex(int i)
    {
        _selected_range = i;
    }

protected:

    int getSliderPosition(int index)
    {
        if (index < 0)
        {
            return 0;
        }

        if (index >= (int)_tick_positions.size())
        {
            return width();
        }

        return QStyle::sliderPositionFromValue(
                   minimum(), maximum(), _tick_positions[index] * (maximum() - minimum()), width());
    }

    void paintEvent(QPaintEvent *ev)
    {
        QPainter painter(this);

        if (_selected_range >= 0)
        {
            int pos1 = getSliderPosition(_selected_range - 1);
            int pos2 = getSliderPosition(_selected_range);
            painter.fillRect(pos1, 0, pos2 - pos1, height(), QColor("#90EE90"));
        }

        for (int i = 0; i < (int)_tick_positions.size(); i++)
        {
            int position = getSliderPosition(i);
            painter.drawLine(position, 0, position, height());
        }

        QSlider::paintEvent(ev);
    }

private:
    int _selected_range;
    std::vector<double> _tick_positions; // from 0.0 to 1.0
};
