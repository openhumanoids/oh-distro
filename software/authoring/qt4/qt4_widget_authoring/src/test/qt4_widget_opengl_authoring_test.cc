#include <iostream>
#include <QtGui/QApplication>

#include "authoring/qt4_widget_opengl_authoring.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_OpenGL_Authoring class demo program" << endl;
  QApplication app( argc, argv );
  
  Qt4_Widget_OpenGL_Authoring qt4_widget_opengl_authoring;
  qt4_widget_opengl_authoring.show();

  return app.exec();
}
