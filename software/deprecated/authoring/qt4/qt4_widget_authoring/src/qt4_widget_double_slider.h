#ifndef AUTHORING_QT4_WIDGET_DOUBLE_SLIDER_H
#define AUTHORING_QT4_WIDGET_DOUBLE_SLIDER_H

#include <iostream>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsRectItem>

namespace authoring {
  class Qt4_Widget_Double_Slider_Handle : public QGraphicsObject {
    Q_OBJECT
  public:
    Qt4_Widget_Double_Slider_Handle( const QString& id = QString( "N/A" ), const QPointF& position = QPointF( 0, 0 ), unsigned int min = 0, unsigned int max = 200, unsigned int width = 20, unsigned int height = 20, QGraphicsItem * parent = 0 );
    ~Qt4_Widget_Double_Slider_Handle();
    Qt4_Widget_Double_Slider_Handle( const Qt4_Widget_Double_Slider_Handle& other );
    Qt4_Widget_Double_Slider_Handle& operator=( const Qt4_Widget_Double_Slider_Handle& other );
  
    void paint( QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget * widget = 0 );  
    QRectF boundingRect( void )const;
  
    void mousePressEvent( QGraphicsSceneMouseEvent * event );
    void mouseMoveEvent( QGraphicsSceneMouseEvent * event );
    void mouseReleaseEvent( QGraphicsSceneMouseEvent * event );

    inline const QString& id( void )const{ return _id; };
    inline unsigned int& min( void ){ return _min; };
    inline unsigned int& max( void ){ return _max; };
    inline unsigned int& width( void ){ return _width; };
    inline unsigned int& height( void ){ return _height; };

  signals:
    void bounds_update( void );

  protected:
    QString _id;
    unsigned int _min;
    unsigned int _max;
    unsigned int _width;
    unsigned int _height;

  private:
  };

  class Qt4_Widget_Double_Slider : public QGraphicsView {
    Q_OBJECT
  public:
    Qt4_Widget_Double_Slider( QWidget * parent = 0 );
    ~Qt4_Widget_Double_Slider();
    Qt4_Widget_Double_Slider( const Qt4_Widget_Double_Slider& other );
    Qt4_Widget_Double_Slider& operator=( const Qt4_Widget_Double_Slider& other );

    inline const double& min( void )const{ return _min; };
    inline const double& max( void )const{ return _max; };

    inline Qt4_Widget_Double_Slider_Handle * start( void ){ return _graphics_item_start; };
    inline Qt4_Widget_Double_Slider_Handle * end( void ){ return _graphics_item_end; };

  signals:
    void start_update( double index );
    void end_update( double index );

  public slots:
    void update_bounds( void );

  protected:
    double _min;
    double _max;  

    QGraphicsScene * _graphics_scene;
    Qt4_Widget_Double_Slider_Handle * _graphics_item_start;
    Qt4_Widget_Double_Slider_Handle * _graphics_item_middle;
    Qt4_Widget_Double_Slider_Handle * _graphics_item_end;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Qt4_Widget_Double_Slider& other );
}

#endif /* AUTHORING_QT4_WIDGET_DOUBLE_SLIDER_H */
