#include <QtGui/QGraphicsSceneMouseEvent>

#include "authoring/qt4_widget_double_slider.h"

using namespace std;
using namespace authoring;

Qt4_Widget_Double_Slider_Handle::
Qt4_Widget_Double_Slider_Handle( const QString& id,
                                  const QPointF& position,
                                  unsigned int min,
                                  unsigned int max,
                                  unsigned int width,
                                  unsigned int height,
                                  QGraphicsItem* parent ) : QGraphicsObject( parent ),
                                                            _id( id ),
                                                            _min( min ),
                                                            _max( max ),
                                                            _width( width ),
                                                            _height( height ) {
  setPos( position );
  setY( y() + _height/2 );
}

Qt4_Widget_Double_Slider_Handle::
~Qt4_Widget_Double_Slider_Handle(){

}

Qt4_Widget_Double_Slider_Handle::
Qt4_Widget_Double_Slider_Handle( const Qt4_Widget_Double_Slider_Handle& other ){

}

Qt4_Widget_Double_Slider_Handle&
Qt4_Widget_Double_Slider_Handle::
operator=( const Qt4_Widget_Double_Slider_Handle& other ){
  return (*this);
}

void
Qt4_Widget_Double_Slider_Handle::
paint( QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget * widget ){
  painter->setBrush( Qt::black );
  painter->drawRect( -( int )( _width )/2, -( int )( _height )/2, _width, _height );
  return;
}

QRectF
Qt4_Widget_Double_Slider_Handle::
boundingRect( void )const{
  return QRectF( -( int )( _width )/2, -( int )( _height )/2, _width, _height );
}

void 
Qt4_Widget_Double_Slider_Handle::
mousePressEvent( QGraphicsSceneMouseEvent * event ){
  setCursor( Qt::ClosedHandCursor );
  update();
  return;
}

void 
Qt4_Widget_Double_Slider_Handle::
mouseMoveEvent( QGraphicsSceneMouseEvent * event ){
  if( ( x() + event->pos().x() ) > _max ){
    setX( _max );
  } else if( ( x() + event->pos().x() ) < _min ) {
    setX( _min );
  } else {
    setX( x() + event->pos().x() );
  }
  emit bounds_update();
  update();
  return;
}

void 
Qt4_Widget_Double_Slider_Handle::
mouseReleaseEvent( QGraphicsSceneMouseEvent * event ){
  setCursor( Qt::ArrowCursor );
  return;
}

Qt4_Widget_Double_Slider::
Qt4_Widget_Double_Slider( QWidget * parent ) : QGraphicsView( parent ),
                                                _min( 0 ),
                                                _max( 400 ),
                                                _graphics_scene( new QGraphicsScene( QRect( 0, 0, _max, 20 ) ) ),
                                                _graphics_item_start( new Qt4_Widget_Double_Slider_Handle( QString( "start" ), QPointF( 0, 0 ), 0, _max / 4 ) ),
                                                _graphics_item_middle( new Qt4_Widget_Double_Slider_Handle( QString( "middle" ), QPointF( 0, 0 ), 0, _max / 4 ) ),
                                                _graphics_item_end( new Qt4_Widget_Double_Slider_Handle( QString( "end" ), QPointF( _max / 4, 0 ), 0, _max ) ) {
  setFixedSize( _max + 50, 30 );
  setScene( _graphics_scene );

  _graphics_item_middle->setX( ( _graphics_item_start->x() + _graphics_item_end->x() ) / 2.0 );
  _graphics_item_middle->width() = _graphics_item_end->x() - _graphics_item_start->x();
  _graphics_item_middle->height() = 4;

  scene()->addItem( _graphics_item_middle );
  scene()->addItem( _graphics_item_start );
  scene()->addItem( _graphics_item_end ); 
  setRenderHint( QPainter::Antialiasing ); 

  connect( _graphics_item_start, SIGNAL( bounds_update() ), this, SLOT( update_bounds() ) );
  connect( _graphics_item_middle, SIGNAL( bounds_update() ), this, SLOT( update_bounds() ) );
  connect( _graphics_item_end, SIGNAL( bounds_update() ), this, SLOT( update_bounds() ) );
}

Qt4_Widget_Double_Slider::
~Qt4_Widget_Double_Slider() {

}

Qt4_Widget_Double_Slider::
Qt4_Widget_Double_Slider( const Qt4_Widget_Double_Slider& other ) {

}

Qt4_Widget_Double_Slider&
Qt4_Widget_Double_Slider::
operator=( const Qt4_Widget_Double_Slider& other ) {

  return (*this);
}

void
Qt4_Widget_Double_Slider::
update_bounds( void ){
  _graphics_item_start->max() = _graphics_item_end->x();
  _graphics_item_middle->setX( ( _graphics_item_start->x() + _graphics_item_end->x() ) / 2.0 ); 
  _graphics_item_middle->width() = _graphics_item_end->x() - _graphics_item_start->x();
  _graphics_item_end->min() = _graphics_item_start->x();
  emit start_update( _graphics_item_start->x() );
  emit end_update( _graphics_item_end->x() );
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Double_Slider& other ) {
    return out;
  }

}
