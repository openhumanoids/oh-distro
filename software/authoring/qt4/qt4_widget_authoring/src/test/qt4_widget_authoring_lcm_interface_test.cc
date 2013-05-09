#include <iostream>
#include <QtGui/QApplication>

#include "authoring/qt4_widget_authoring_lcm_interface.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_Authoring_LCM_Interface class demo program" << endl;
  
  QApplication app( argc, argv );

  Qt4_Widget_Authoring_LCM_Interface qt4_widget_authoring_lcm_interface;
  qt4_widget_authoring_lcm_interface.show();

  return app.exec();
}
