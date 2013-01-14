#include <iostream>
#include <QtGui/QApplication>

#include "qt4/qt4_widget_gfe_object.h"
#include "qt4/qt4_widget_gfe_control.h"

using namespace std;
using namespace qt4;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_GFE_Object class demo program" << endl;
  QApplication app( argc, argv );

  Qt4_Widget_GFE_Object qt4_widget_gfe_object;
  qt4_widget_gfe_object.show();  

  Qt4_Widget_GFE_Control qt4_widget_gfe_control;
  qt4_widget_gfe_control.show();

  QObject::connect( &qt4_widget_gfe_control, SIGNAL( state_update( state::State_GFE& ) ), &qt4_widget_gfe_object, SLOT( update_state( state::State_GFE& ) ) );

  return app.exec();
}
