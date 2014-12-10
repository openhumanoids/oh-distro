#include <iostream>
#include <QtGui/QApplication>

#include "qt4/qt4_widget_gfe_control.h"

using namespace std;
using namespace qt4;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_GFE_Control class demo program" << endl;
  QApplication app( argc, argv );
  
  Qt4_Widget_GFE_Control qt4_widget_gfe_control;
  qt4_widget_gfe_control.show();

  return app.exec();
}
