#include <iostream>
#include <QtGui/QApplication>

#include <path_util/path_util.h>

#include "qt4/qt4_widget_gfe_object.h"
#include "qt4/qt4_widget_gfe_control.h"

using namespace std;
using namespace kinematics;
using namespace qt4;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Qt4_Widget_GFE_Object class demo program" << endl;
  QApplication app( argc, argv );

//  std::string urdf_filename = getModelsPath() + string( "/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf" );
//  std::string urdf_filename = getModelsPath() + string( "/mit_gazebo_models/mit_robot_drake/model_simple_visuals_minimal_contact_point_hands.urdf" );
  std::string urdf_filename = getModelsPath() + string( "/mit_gazebo_models/mit_robot/model.urdf" );

  Qt4_Widget_GFE_Object qt4_widget_gfe_object( Kinematics_Model_GFE::urdf_filename_to_xml_string( urdf_filename ) );
  qt4_widget_gfe_object.show();  

  Qt4_Widget_GFE_Control qt4_widget_gfe_control;
  qt4_widget_gfe_control.show();

  QObject::connect( &qt4_widget_gfe_control, SIGNAL( state_update( state::State_GFE& ) ), &qt4_widget_gfe_object, SLOT( update_state( state::State_GFE& ) ) );

  return app.exec();
}
