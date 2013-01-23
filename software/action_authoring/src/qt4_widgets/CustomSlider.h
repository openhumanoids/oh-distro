// derived from http://stackoverflow.com/questions/9802670/how-to-add-a-tick-mark-to-a-slider-if-it-cannot-inherit-qslider

#include <QtGui>

class DefaultValueSlider : public QSlider {
  Q_OBJECT

 public:
  DefaultValueSlider(Qt::Orientation orientation, QWidget *parent = NULL)
    : QSlider(orientation, parent),
      _tick_positions() {
    connect(this, SIGNAL(valueChanged(int)), SLOT(VerifyDefaultValue(int)));
  }

 protected:
  void paintEvent(QPaintEvent *ev) {
    QPainter painter(this);
    for (int i = 0; _tick_positions.size(); i++) {
	int position = QStyle::sliderPositionFromValue(
	    minimum(), maximum(), _tick_positions[i], width());
	painter.drawLine(position, 0, position, height());
    }
    QSlider::paintEvent(ev);
  }

 private slots:
  void VerifyDefaultValue(int value){
    if (_default_value == -1) {
      _default_value = value;
      update();
    }
  }

 private:
  int _default_value;
  vector<double> _tick_positions; // from 0.0 to 1.0
};
