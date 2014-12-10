#include <iostream>
#include <QtGui/QApplication>

#include "kinematics/kinematics_model_gfe.h"
#include "authoring/qt4_widget_authoring.h"

using namespace std;
using namespace kinematics;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_Authoring class demo program" << endl;
  QApplication app( argc, argv );
  
  string xml_string = Kinematics_Model_GFE::urdf_filename_to_xml_string( getModelsPath() + string( "/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf" ) ); 

  Qt4_Widget_Authoring qt4_widget_authoring( xml_string );
  qt4_widget_authoring.show();

  return app.exec();
}
