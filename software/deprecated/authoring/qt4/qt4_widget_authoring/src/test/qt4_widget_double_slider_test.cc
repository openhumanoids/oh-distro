#include <iostream>
#include <QtGui/QApplication>

#include "authoring/qt4_widget_double_slider.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_Double_Slider class demo program" << endl;
  QApplication app( argc, argv );

  Qt4_Widget_Double_Slider qt4_widget_double_slider;
  qt4_widget_double_slider.show();

  return app.exec();
}
