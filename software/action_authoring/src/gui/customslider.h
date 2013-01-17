// derived from http://stackoverflow.com/questions/9802670/how-to-add-a-tick-mark-to-a-slider-if-it-cannot-inherit-qslider

#include <QtGui>

class DefaultValueSlider : public QSlider {
  Q_OBJECT

 public:
  DefaultValueSlider(Qt::Orientation orientation, QWidget *parent = NULL)
    : QSlider(orientation, parent),
      default_value_(-1) {
    connect(this, SIGNAL(valueChanged(int)), SLOT(VerifyDefaultValue(int)));
  }

 protected:
  void paintEvent(QPaintEvent *ev) {
    int position = QStyle::sliderPositionFromValue(minimum(),
                                                   maximum(),
                                                   default_value_,
                                                   width());
    QPainter painter(this);
    painter.drawLine(position, 0, position, height());
    QSlider::paintEvent(ev);
  }

 private slots:
  void VerifyDefaultValue(int value){
    if (default_value_ == -1) {
      default_value_ = value;
      update();
    }
  }

 private:
  int default_value_;
};
