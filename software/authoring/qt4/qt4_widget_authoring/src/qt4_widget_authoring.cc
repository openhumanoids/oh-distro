#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QSplitter>
#include <QFileDialog>
#include <kinematics/kinematics_model_gfe.h>

#include "authoring/qt4_widget_authoring.h"

using namespace std;
using namespace opengl;
using namespace qt4;
using namespace KDL;
using namespace drc;
using namespace kinematics;
using namespace state;
using namespace affordance;
using namespace authoring;

Qt4_Widget_Authoring::
Qt4_Widget_Authoring( const std::string& xmlString, 
                      unsigned int numConstraints,
                      QWidget * parent ) : QWidget( parent ),
                                           _widget_opengl_authoring( new Qt4_Widget_OpenGL_Authoring( xmlString, this ) ),
                                            _text_edit_info( new QTextEdit( "[<b>OK</b>] authoring widget started", this ) ),
                                            _push_button_grab( new QPushButton( QString( "grab" ), this ) ),
                                            _push_button_import( new QPushButton( QString( "import..." ), this ) ),
                                            _push_button_export( new QPushButton( QString( "export..." ), this ) ),
                                            _push_button_publish( new QPushButton( QString( "publish constraints" ), this ) ),
                                            _push_button_stand_up_from_back( new QPushButton( QString( "stand up from back" ), this ) ),
                                            _push_button_stand_up_from_front( new QPushButton( QString( "stand up from front" ), this ) ),
                                            _text_edit_affordance_collection( new QTextEdit( "N/A", this ) ),
                                            _slider_plan_current_index( new QSlider( Qt::Horizontal, this ) ),
                                            _check_box_visible_current_index( new QCheckBox( "current index", this ) ),
                                            _check_box_visible_trajectory( new QCheckBox( "trajectory", this ) ),
                                            _check_box_visible_trajectory_wrist( new QCheckBox( "wrist trajectory", this ) ),
                                            _check_box_visible_initial_state( new QCheckBox( "initial state", this ) ),
                                            _slider_current_time( new QLabel("frame 0") ),
                                            _robot_model(),
                                            _kinematics_model_gfe( xmlString ),
                                            _affordance_collection(),
                                            _affordance_collection_ghost(),
                                            _robot_plan(),
                                            _state_gfe_ghost(),
                                            _constraint_sequence( numConstraints ){
  _robot_model.initString( xmlString );

  for( unsigned int i = 0; i < numConstraints; i++ ){
    _constraint_sequence.constraints()[ i ].id() = QString( "C%1" ).arg( i ).toStdString();
    _constraint_sequence.constraints()[ i ].active() = false;
    _constraint_editors.push_back( new Qt4_Widget_Constraint_Editor( _constraint_sequence.constraints()[ i ], _robot_model, _affordance_collection, xmlString, i, this ) );
  }

  _push_button_publish->setToolTip("publish all constraints to a motion plan server over LCM");
  _push_button_grab->setToolTip("make all affordances and the robot currently in the scene available as \"child\" links for constraints");
  _push_button_import->setToolTip("import all constraints from a file");
  _push_button_export->setToolTip("export all constraints to a file");

  _text_edit_info->setFixedHeight( 75 );

  _text_edit_info->setTextInteractionFlags(Qt::TextSelectableByMouse);
  _text_edit_affordance_collection->setTextInteractionFlags(Qt::TextSelectableByMouse);

  _check_box_visible_current_index->setCheckState( Qt::Checked );
  _check_box_visible_trajectory->setCheckState( Qt::Checked );
  _check_box_visible_trajectory_wrist->setCheckState( Qt::Unchecked );
  _check_box_visible_initial_state->setCheckState( Qt::Checked );

  QGroupBox * controls_group_box = new QGroupBox( QString( "controls" ) );
  QGridLayout * controls_layout = new QGridLayout();
  controls_layout->addWidget( _push_button_grab, 0, 0 );
  controls_layout->addWidget( _push_button_import, 0, 1 );
  controls_layout->addWidget( _push_button_export, 0, 2 );
  controls_layout->addWidget( _push_button_publish, 0, 3 );
  controls_layout->addWidget( _push_button_stand_up_from_back, 1, 0 );
  controls_layout->addWidget( _push_button_stand_up_from_front, 1, 1 );
  controls_group_box->setLayout( controls_layout );
 
  QWidget * affordances_widget = new QWidget( this );
  QGridLayout * affordances_layout = new QGridLayout();
  affordances_layout->addWidget( _text_edit_affordance_collection );
  affordances_widget->setLayout( affordances_layout );
 
  QScrollArea * constraints_scroll_area = new QScrollArea( this );
  constraints_scroll_area->setFrameStyle( QFrame::NoFrame );
  QWidget * constraints_widget = new QWidget( this );
  QGridLayout * constraints_layout = new QGridLayout();
  for( vector< Qt4_Widget_Constraint_Editor* >::iterator it = _constraint_editors.begin(); it != _constraint_editors.end(); it++ ){
    constraints_layout->addWidget( *it );
  }
  constraints_widget->setLayout( constraints_layout );
  constraints_scroll_area->setWidget( constraints_widget ); 
  
  QScrollArea * plan_scroll_area = new QScrollArea( this );
  plan_scroll_area->setFrameStyle( QFrame::NoFrame );
  QWidget * plan_widget = new QWidget( this );
  QGridLayout * plan_layout =  new QGridLayout();
  plan_layout->addWidget( new QLabel( QString("visualize: ") ), 0, 0 );
  plan_layout->addWidget( _check_box_visible_initial_state, 0, 1 );
  plan_layout->addWidget( _check_box_visible_current_index, 0, 2 );
  plan_layout->addWidget( _check_box_visible_trajectory, 0, 3 );
  plan_layout->addWidget( _check_box_visible_trajectory_wrist, 0, 4 );
  plan_layout->addWidget( _slider_plan_current_index, 1, 0, 1, 4 );
  plan_layout->addWidget( _slider_current_time, 2, 0, 1, 5 );
  plan_widget->setLayout( plan_layout );

  plan_scroll_area->setWidget( plan_widget );

  QTabWidget * tab_widget = new QTabWidget( this );
  tab_widget->addTab( constraints_scroll_area, QString( "constraints" ) ); 
  tab_widget->addTab( plan_scroll_area, QString( "plan" ) ); 
  tab_widget->addTab( affordances_widget, QString( "affordances" ) ); 


  QVBoxLayout * widget_layout_lower = new QVBoxLayout();
  widget_layout_lower->addWidget( _text_edit_info );
  widget_layout_lower->addWidget( controls_group_box );
  widget_layout_lower->addWidget( tab_widget );
  QWidget * widget_lower = new QWidget();
  widget_lower->setLayout( widget_layout_lower );

  QSplitter *splitter = new QSplitter();

  splitter->addWidget( _widget_opengl_authoring );
  splitter->addWidget( widget_lower );
  splitter->setOrientation( Qt::Vertical );
  splitter->setStretchFactor( 0, 1 ); // give the top as much space as possible
  splitter->setStretchFactor( 1, 0 ); // give the bottom a stretch factor of 0

  QGridLayout * widget_layout = new QGridLayout();
  widget_layout->addWidget(splitter);
  setLayout( widget_layout );

  connect( this, SIGNAL( affordance_collection_update( std::vector< affordance::AffordanceState >& ) ),
              _widget_opengl_authoring, SLOT( update_opengl_object_affordance_collection( std::vector< affordance::AffordanceState >& ) ) );
  connect( this, SIGNAL( state_gfe_update( state::State_GFE& ) ), _widget_opengl_authoring, SLOT( update_opengl_object_gfe( state::State_GFE& ) ) );
  connect( _slider_plan_current_index, SIGNAL( valueChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_current_index( int ) ) );
  connect( _slider_plan_current_index, SIGNAL( valueChanged( int ) ), this, SLOT( _slider_updated( int ) ) );
  connect( _check_box_visible_current_index, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_current_index( int ) ) );
  connect( _check_box_visible_trajectory, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_trajectory( int ) ) );
  connect( _check_box_visible_trajectory_wrist, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_trajectory_wrist( int ) ) );
  connect( _check_box_visible_initial_state, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_initial_state( int ) ) );
  connect( this, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
  for( vector< Qt4_Widget_Constraint_Editor* >::iterator it = _constraint_editors.begin(); it != _constraint_editors.end(); it++ ){
    connect( *it, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
    connect( *it, SIGNAL( constraint_update( const Constraint_Task_Space_Region&, unsigned int ) ), this, SLOT( update_constraint( const Constraint_Task_Space_Region&, unsigned int ) ) );
    connect( *it, SIGNAL( constraint_highlight ( const QString& ) ), _widget_opengl_authoring, SLOT( highlight_constraint( const QString& ) ) );
  }
  connect( _push_button_grab, SIGNAL( clicked() ), this, SLOT( _push_button_grab_pressed() ) );
  connect( _push_button_import, SIGNAL( clicked() ), this, SLOT( _push_button_import_pressed() ) );
  connect( _push_button_export, SIGNAL( clicked() ), this, SLOT( _push_button_export_pressed() ) );
  connect( _push_button_publish, SIGNAL( clicked() ), this, SLOT( _push_button_publish_pressed() ) );
  connect( _push_button_stand_up_from_back, SIGNAL( clicked() ), this, SLOT( _push_button_stand_up_from_back_pressed() ) );
  connect( _push_button_stand_up_from_front, SIGNAL( clicked() ), this, SLOT( _push_button_stand_up_from_front_pressed() ) );
}

Qt4_Widget_Authoring::
~Qt4_Widget_Authoring() {

}

Qt4_Widget_Authoring::
Qt4_Widget_Authoring( const Qt4_Widget_Authoring& other ) : QWidget(),
                                                            _widget_opengl_authoring( new Qt4_Widget_OpenGL_Authoring( "n/a", this ) ) {

}

Qt4_Widget_Authoring&
Qt4_Widget_Authoring::
operator=( const Qt4_Widget_Authoring& other ) {
  _widget_opengl_authoring = other._widget_opengl_authoring;
  return (*this);
}

void
Qt4_Widget_Authoring::
update_info( const QString& info ){
  _text_edit_info->append( info );
  _text_edit_info->moveCursor( QTextCursor::End );
  update();
  return;
}

void
Qt4_Widget_Authoring::
update_constraint( const Constraint_Task_Space_Region& constraint,
                    unsigned int constraintIndex ){
  if( constraintIndex < _constraint_sequence.constraints().size() ){
    _constraint_sequence.constraints()[ constraintIndex ] = constraint;
    _widget_opengl_authoring->update_opengl_object_constraint_sequence( _constraint_sequence );
  }
  return;
}

void 
Qt4_Widget_Authoring::
update_affordance_collection( vector< AffordanceState >& affordanceCollection ){
  _affordance_collection_ghost = affordanceCollection;
  return;
}

void
Qt4_Widget_Authoring::
update_robot_plan( vector< State_GFE >& robotPlan ){
  emit info_update( QString( "[<b>OK</b>] received plan" ) );
  _robot_plan = robotPlan;
  _slider_plan_current_index->setRange( 0, ( robotPlan.size() - 1 ) );
  return;
}

void
Qt4_Widget_Authoring::
update_state_gfe( State_GFE& stateGFE ){
  _state_gfe_ghost = stateGFE;
  return;
} 

void
Qt4_Widget_Authoring::
highlight_constraint( const QString& id ){
  cout << "Qt4_Widget_Authoring::highlight_constraint: " << id.toStdString() << endl;
  return;
}

void
Qt4_Widget_Authoring::
_push_button_grab_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] grab pressed" ) );
  _affordance_collection = _affordance_collection_ghost;
  _text_edit_affordance_collection->clear();
  for( vector< AffordanceState >::iterator it = _affordance_collection.begin(); it != _affordance_collection.end(); it++ ){
    _text_edit_affordance_collection->append( QString( "name: %1 uid: <%2,%3> xyz: (%4,%5,%6) rpy: (%7,%8,%9)" ).arg( QString::fromStdString( it->getName() ) ).arg( it->getGlobalUniqueId().first ).arg( it->getGlobalUniqueId().second ).arg( QString::number( it->getXYZ().x() ) ).arg( QString::number( it->getXYZ().y() ) ).arg( QString::number( it->getXYZ().z() ) ).arg( QString::number( it->getRPY().x() ) ).arg( QString::number( it->getRPY().y() ) ).arg( QString::number( it->getRPY().z() ) ) );
  }
  _constraint_sequence.q0() = _state_gfe_ghost;
  emit affordance_collection_update( _affordance_collection );
  emit state_gfe_update( _constraint_sequence.q0() );
  return;
}

void
Qt4_Widget_Authoring::
_push_button_import_pressed( void ){
  QString filename = QFileDialog::getOpenFileName(this, tr("Load Constraint Sequence"),
                                                  "/home",
                                                  tr("ActionSequence (*.xml)"));  
  if (filename.isEmpty()) {
    emit info_update( QString( "[<b>ERROR</b>] failed to import (filename empty)" ) );
    return;
  }

  _constraint_sequence.from_xml( filename.toStdString() );
  emit info_update( QString( "[<b>OK</b>] imported constraint sequence to file %1" ).arg( filename ) );
  for( unsigned int i = 0; i < _constraint_editors.size(); i++ ){
    _constraint_editors[ i ]->update_constraint( _constraint_sequence.constraints()[ i ] );
  } 
  return;
}

void
Qt4_Widget_Authoring::
_push_button_export_pressed( void )
{
  QString filename = QFileDialog::getSaveFileName(this, tr("Save Constraint Sequence"),
                                                  "/home/untitled.xml",
                                                  tr("ActionSequence (*.xml)"));
  
  if ( filename.isEmpty() ){
    emit info_update( QString( "[<b>ERROR</b>] failed to export (filename empty)" ) );
    return;
  }

  _constraint_sequence.to_xml( filename.toStdString() );
  emit info_update( QString( "[<b>OK</b>] exported constraint sequence to file %1" ).arg( filename ) );

  return;
}

void
Qt4_Widget_Authoring::
_push_button_publish_pressed( void ){
  action_sequence_t msg; 
  _constraint_sequence.to_msg( msg, _affordance_collection ); 
  Constraint_Sequence::print_msg( msg );
  emit drc_action_sequence_t_publish( msg );     
  emit info_update( QString( "[<b>OK</b>] published constraint sequence as drc::action_sequence_t" ) ); 
  return;
}

void
Qt4_Widget_Authoring::
_push_button_stand_up_from_back_pressed( void ){
  for( unsigned int i = 0; i < _constraint_sequence.constraints().size(); i++ ){
    _constraint_sequence.constraints()[i].active() = false;
  }

  AffordanceState * ground = NULL;
  for( unsigned int i = 0; i < _affordance_collection.size(); i++ ){
    cout << "affordance_collection[" << i << "]: " << _affordance_collection[ i ].getName() << " xyz: " << _affordance_collection[ i ].getXYZ().x() << "," << _affordance_collection[ i ].getXYZ().y() << "," << _affordance_collection[ i ].getXYZ().z() << endl;
  }
  if( _affordance_collection.size() == 1 ){
    ground = &_affordance_collection[ 0 ];
  }

  _kinematics_model_gfe.set( _constraint_sequence.q0() );
  Frame frame_pelvis = _kinematics_model_gfe.link( "pelvis" );
  Frame frame_head = _kinematics_model_gfe.link( "head" );
  double yaw = atan2( frame_head.p[1] - frame_pelvis.p[1], frame_head.p[0] - frame_pelvis.p[0] );

  Frame frame_constraint_center = Frame( Rotation::RPY( 0.0, 0.0, yaw ), Vector( frame_pelvis.p[0], frame_pelvis.p[1], 0.0 ) );
/*
  // initial constraints
  _constraint_sequence.constraints()[ 4 ] = new Constraint_Task_Space_Region( "4", true, 0.1, 1.0, "utorso-back", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 4 ] )->ranges()[ 0 ].second = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 4 ] )->ranges()[ 1 ].second = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 4 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 ); 

  _constraint_sequence.constraints()[ 5 ] = new Constraint_Task_Space_Region( "5", true, 0.1, 1.0, "pelvis-back", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 5 ] )->ranges()[ 0 ].second = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 5 ] )->ranges()[ 1 ].second = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 5 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 );

  // put hands and feet on the floor
  Frame frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( -0.6, -0.2, 0.0 ) );
  _constraint_sequence.constraints()[ 10 ] = new Constraint_Task_Space_Region( "10", true, 0.1, 1.0, "l_foot-toe", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 0 ].second = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 1 ].second = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 );
  _constraint_sequence.constraints()[ 11 ] = new Constraint_Task_Space_Region( "11", true, 0.1, 1.0, "l_foot-heel", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 0 ].second = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 1 ].second = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 );

  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( -0.6, 0.2, 0.0 ) );
  _constraint_sequence.constraints()[ 12 ] = new Constraint_Task_Space_Region( "12", true, 0.1, 1.0, "r_foot-toe", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 12 ] )->ranges()[ 0 ].second = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 12 ] )->ranges()[ 1 ].second = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 12 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 );
  _constraint_sequence.constraints()[ 13 ] = new Constraint_Task_Space_Region( "13", true, 0.1, 1.0, "r_foot-heel", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 13 ] )->ranges()[ 0 ].second = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 13 ] )->ranges()[ 1 ].second = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 13 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 );

  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( 0.5, -0.4, 0.0 ) );
  _constraint_sequence.constraints()[ 14 ] = new Constraint_Task_Space_Region( "14", true, 0.1, 1.0, "left_palm-knuckle", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 14 ] )->ranges()[ 0 ].second = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 14 ] )->ranges()[ 1 ].second = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 14 ] )->ranges()[ 2 ].second = pair< double, double >( 0.0, 0.0 );
  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( 0.5, 0.4, 0.0 ) );
  _constraint_sequence.constraints()[ 15 ] = new Constraint_Task_Space_Region( "15", true, 0.1, 1.0, "right_palm-knuckle", ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 15 ] )->ranges()[ 0 ] = pair< bool, pair< double, double > >( true, pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 ) );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 15 ] )->ranges()[ 1 ] = pair< bool, pair< double, double > >( true, pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 ) );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 15 ] )->ranges()[ 2 ] = pair< bool, pair< double, double > >( true, pair< double, double >( 0.0, 0.0 ) );
*/
  for( unsigned int i = 0; i < _constraint_sequence.constraints().size(); i++ ){
    _constraint_editors[ i ]->update_constraint( _constraint_sequence.constraints()[ i ] );
  }

  return;
}

void
Qt4_Widget_Authoring::
_push_button_stand_up_from_front_pressed( void ){
  for( unsigned int i = 0; i < _constraint_sequence.constraints().size(); i++ ){
    _constraint_sequence.constraints()[i].active() = false;
  }


  AffordanceState * ground = NULL;
  for( unsigned int i = 0; i < _affordance_collection.size(); i++ ){
    cout << "affordance_collection[" << i << "]: " << _affordance_collection[ i ].getName() << " xyz: " << _affordance_collection[ i ].getXYZ().x() << "," << _affordance_collection[ i ].getXYZ().y() << "," << _affordance_collection[ i ].getXYZ().z() << endl;
  }
  if( _affordance_collection.size() == 1 ){
    ground = &_affordance_collection[ 0 ];
  }

  cout << "state_gfe xyz: " << _constraint_sequence.q0().pose().p[0] << "," << _constraint_sequence.q0().pose().p[1] << "," << _constraint_sequence.q0().pose().p[2] << endl;
  _kinematics_model_gfe.set( _constraint_sequence.q0() );
  Frame frame_pelvis = _kinematics_model_gfe.link( "pelvis" );
  cout << "pelvis xyz: " << frame_pelvis.p[0] << "," << frame_pelvis.p[1] << "," << frame_pelvis.p[2] << endl;
  Frame frame_head = _kinematics_model_gfe.link( "head" );
  cout << "head xyz: " << frame_head.p[0] << "," << frame_head.p[1] << "," << frame_head.p[2] << endl;

  double yaw = atan2( frame_head.p[1] - frame_pelvis.p[1], frame_head.p[0] - frame_pelvis.p[0] );
  cout << "yaw: " << yaw << endl;

  Frame frame_constraint_center = Frame( Rotation::RPY( 0.0, 0.0, yaw ), Vector( frame_pelvis.p[0], frame_pelvis.p[1], 0.0 ) );
  cout << "frame_constraint_center xyz: " << frame_constraint_center.p[0] << "," << frame_constraint_center.p[1] << "," << frame_constraint_center.p[2] << endl;
/*
  // z > 0 constraints
  _constraint_sequence.constraints()[ 0 ] = new Constraint_Task_Space_Region( "0", true, 0.1, 1.0, pair< string, string >( "l_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 0 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 0 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 0 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 100.0 );
  _constraint_sequence.constraints()[ 1 ] = new Constraint_Task_Space_Region( "1", true, 0.1, 1.0, pair< string, string >( "l_foot", "heel" ), ground, CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 1 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 1 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 1 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 100.0 );
  _constraint_sequence.constraints()[ 2 ] = new Constraint_Task_Space_Region( "2", true, 0.1, 1.0, pair< string, string >( "r_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 2 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 2 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 2 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 100.0 );
  _constraint_sequence.constraints()[ 3 ] = new Constraint_Task_Space_Region( "3", true, 0.1, 1.0, pair< string, string >( "r_foot", "heel" ), ground, CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 3 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 3 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 3 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 100.0 );

  _constraint_sequence.constraints()[ 4 ] = new Constraint_Task_Space_Region( "4", true, 0.1, 1.0, pair< string, string >( "utorso", "front" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 4 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 4 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 4 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );

  _constraint_sequence.constraints()[ 5 ] = new Constraint_Task_Space_Region( "5", true, 0.1, 1.0, pair< string, string >( "l_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 5 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 5 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 5 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );

  _constraint_sequence.constraints()[ 6 ] = new Constraint_Task_Space_Region( "6", true, 0.1, 1.0, pair< string, string >( "r_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 6 ] )->ranges()[ 0 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 6 ] )->ranges()[ 1 ] = pair< double, double >( -10000.0, 10000.0 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 6 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );

  // put hands on the floor
  Frame frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( 0.75, 0.5, 0.0 ) );
  _constraint_sequence.constraints()[ 10 ] = new Constraint_Task_Space_Region( "10", true, 0.1, 1.0, pair< string, string >( "left_palm", "knuckle" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( 0.75, -0.5, 0.0 ) );
  _constraint_sequence.constraints()[ 11 ] = new Constraint_Task_Space_Region( "11", true, 0.1, 1.0, pair< string, string >( "right_palm", "knuckle" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
*/
/*
  // put hands and feet on the floor
  Frame frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( -0.75, 0.2, 0.0 ) );
  _constraint_sequence.constraints()[ 10 ] = new Constraint_Task_Space_Region( "10", true, 0.1, 1.0, pair< string, string >( "l_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 10 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
  _constraint_sequence.constraints()[ 11 ] = new Constraint_Task_Space_Region( "11", true, 0.1, 1.0, pair< string, string >( "l_foot", "heel" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 11 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );

  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( -0.75, -0.2, 0.0 ) );
  _constraint_sequence.constraints()[ 12 ] = new Constraint_Task_Space_Region( "12", true, 0.1, 1.0, pair< string, string >( "r_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 12 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 12 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 12 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
  _constraint_sequence.constraints()[ 13 ] = new Constraint_Task_Space_Region( "13", true, 0.1, 1.0, pair< string, string >( "r_foot", "heel" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 13 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 13 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 13 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );

  // raise the body
  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( -0.75, 0.2, 0.0 ) );
  _constraint_sequence.constraints()[ 20 ] = new Constraint_Task_Space_Region( "20", true, 0.1, 1.0, pair< string, string >( "l_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 20 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 20 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 20 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
  _constraint_sequence.constraints()[ 21 ] = new Constraint_Task_Space_Region( "21", true, 0.1, 1.0, pair< string, string >( "l_foot", "heel" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 21 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 21 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 21 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );

  frame_tmp = frame_constraint_center * Frame( Rotation::RPY( 0.0, 0.0, 0.0 ), Vector( -0.75, -0.2, 0.0 ) );
  _constraint_sequence.constraints()[ 22 ] = new Constraint_Task_Space_Region( "22", true, 0.1, 1.0, pair< string, string >( "r_foot", "toe" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 22 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 22 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 22 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
  _constraint_sequence.constraints()[ 23 ] = new Constraint_Task_Space_Region( "23", true, 0.1, 1.0, pair< string, string >( "r_foot", "heel" ), ground, CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 23 ] )->ranges()[ 0 ] = pair< double, double >( frame_tmp.p[0] - 0.3, frame_tmp.p[0] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 23 ] )->ranges()[ 1 ] = pair< double, double >( frame_tmp.p[1] - 0.3, frame_tmp.p[1] + 0.3 );
  dynamic_cast< Constraint_Task_Space_Region* >( _constraint_sequence.constraints()[ 23 ] )->ranges()[ 2 ] = pair< double, double >( 0.0, 0.0 );
*/

  for( unsigned int i = 0; i < _constraint_sequence.constraints().size(); i++ ){
    _constraint_editors[ i ]->update_constraint( _constraint_sequence.constraints()[ i ] );
  }

  return;
}

void
Qt4_Widget_Authoring::
_slider_updated( int currentIndex ){
  if ( currentIndex < _robot_plan.size() ) {
    _slider_current_time->setText( QString( "time: %1 sec" ).arg( _robot_plan[currentIndex].time() / 1000000.0 ) );
  }
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Authoring& other ) {
    return out;
  }

}
