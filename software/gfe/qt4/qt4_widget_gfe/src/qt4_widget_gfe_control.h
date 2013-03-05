#ifndef QT4_QT4_WIDGET_GFE_CONTROL_H
#define QT4_QT4_WIDGET_GFE_CONTROL_H

#include <iostream>
#include <map>

#include <QtGui/QLabel>
#include <QtGui/QSlider>
#include <QtGui/QWidget>
#include <QtGui/QTabWidget>

#include <state/state_gfe.h>

namespace qt4 {
  class Qt4_Widget_GFE_Control: public QTabWidget {
    Q_OBJECT
  public:
    Qt4_Widget_GFE_Control( QWidget * parent = 0 );
    ~Qt4_Widget_GFE_Control();
    Qt4_Widget_GFE_Control( const Qt4_Widget_GFE_Control& other );
    Qt4_Widget_GFE_Control& operator=( const Qt4_Widget_GFE_Control& other );

  signals:
    void state_update( state::State_GFE& );

  public slots:
    void update_state( state::State_GFE& );

  protected slots:

    void _update_joints( int );

  protected:
    state::State_GFE _state_gfe;
    std::map< std::string, QSlider* > _sliders;
    std::map< std::string, QLabel* > _values;
  private:


  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_GFE_Control& other );
}

#endif /* QT4_QT4_WIDGET_GFE_CONTROL_H */
