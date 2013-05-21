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

  std::string defaultUrdf = "/mit_gazebo_models/mit_robot_drake/model_simple_visuals_minimal_contact_point_hands.urdf";
  std::string urdfFilename = defaultUrdf;
  if ( argc >= 2 ) {
      urdfFilename = argv[1];
      std::cout << "using non-default urdf from : " << urdfFilename << std::endl;
  }

  Qt4_Widget_Authoring_LCM_Interface qt4_widget_authoring_lcm_interface(urdfFilename);
  qt4_widget_authoring_lcm_interface.show();

  return app.exec();
}
