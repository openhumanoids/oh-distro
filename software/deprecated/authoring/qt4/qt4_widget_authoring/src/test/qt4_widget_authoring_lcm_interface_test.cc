#include <iostream>
#include <QtGui/QApplication>

#include "kinematics/kinematics_model_gfe.h"
#include "authoring/qt4_widget_authoring_lcm_interface.h"

using namespace std;
using namespace kinematics;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_Authoring_LCM_Interface class demo program" << endl;
  
  QApplication app( argc, argv );

  string xml_string = Kinematics_Model_GFE::urdf_filename_to_xml_string( getModelsPath() + string( "/mit_gazebo_models/mit_robot_drake/model_simple_visuals.urdf" ) );

  if ( argc >= 2 ) {
      xml_string = Kinematics_Model_GFE::urdf_filename_to_xml_string( argv[1] ); 
      std::cout << "using non-default urdf from : " << argv[1] << std::endl;
  }

  Qt4_Widget_Authoring_LCM_Interface qt4_widget_authoring_lcm_interface( xml_string );
  qt4_widget_authoring_lcm_interface.show();

  return app.exec();
}
