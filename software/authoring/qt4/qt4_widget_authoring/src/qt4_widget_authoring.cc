#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QGroupBox>
#include <QFileDialog>
#include <kinematics/kinematics_model_gfe.h>

#include "authoring/qt4_widget_authoring.h"

using namespace std;
using namespace opengl;
using namespace qt4;
using namespace drc;
using namespace kinematics;
using namespace state;
using namespace affordance;
using namespace authoring;

Qt4_Widget_Authoring::
Qt4_Widget_Authoring( const std::string& urdfFilename, 
                      unsigned int numConstraints,
                      QWidget * parent ) : QWidget( parent ),
                                            _widget_opengl_authoring( new Qt4_Widget_OpenGL_Authoring( this ) ),
                                            _text_edit_info( new QTextEdit( "[<b>OK</b>] authoring widget started", this ) ),
                                            _push_button_grab( new QPushButton( QString( "grab" ), this ) ),
                                            _push_button_import( new QPushButton( QString( "import" ), this ) ),
                                            _push_button_export( new QPushButton( QString( "export" ), this ) ),
                                            _push_button_publish( new QPushButton( QString( "publish" ), this ) ),
                                            _double_spin_box_end_time( new QDoubleSpinBox( this ) ),
                                            _text_edit_affordance_collection( new QTextEdit( "N/A", this ) ),
                                            _slider_plan_current_index( new QSlider( Qt::Horizontal, this ) ),
                                            _check_box_visible_current_index( new QCheckBox( "current index", this ) ),
                                            _check_box_visible_trajectory( new QCheckBox( "trajectory", this ) ),
                                            _check_box_visible_trajectory_wrist( new QCheckBox( "wrist trajectory", this ) ),
                                            _robot_model(),
                                            _affordance_collection(),
                                            _affordance_collection_ghost(),
                                            _robot_plan(),
                                            _state_gfe(),
                                            _state_gfe_ghost(){
  _robot_model.initString( Kinematics_Model_GFE::urdf_filename_to_xml_string( getModelsPath() + urdfFilename ) );

  _constraints.resize( numConstraints );
  for( unsigned int i = 0; i < numConstraints; i++ ){
    _constraints[ i ] = NULL;
    _constraint_editors.push_back( new Qt4_Widget_Constraint_Editor( _constraints[ i ], _robot_model, _affordance_collection, ( QString( "C%1" ).arg( QString::number( i ) ) ).toStdString(), this ) );
  }

  _text_edit_info->setFixedHeight( 75 );
  _double_spin_box_end_time->setSuffix( QString( " seconds" ) );

  _check_box_visible_current_index->setCheckState( Qt::Checked );
  _check_box_visible_trajectory->setCheckState( Qt::Checked );
  _check_box_visible_trajectory_wrist->setCheckState( Qt::Unchecked );

  QGroupBox * controls_group_box = new QGroupBox( QString( "controls" ) );
  QHBoxLayout * controls_layout = new QHBoxLayout();
  controls_layout->addWidget( _push_button_grab );
  controls_layout->addWidget( _push_button_import );
  controls_layout->addWidget( _push_button_export );
  controls_layout->addWidget( _push_button_publish );
  controls_layout->addWidget( _double_spin_box_end_time );
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
  constraints_scroll_area->setFixedHeight( 160 );
  
  QScrollArea * plan_scroll_area = new QScrollArea( this );
  plan_scroll_area->setFrameStyle( QFrame::NoFrame );
  QWidget * plan_widget = new QWidget( this );
  QGridLayout * plan_layout =  new QGridLayout();
  plan_layout->addWidget( _check_box_visible_current_index, 0, 0 );
  plan_layout->addWidget( _check_box_visible_trajectory, 0, 1 );
  plan_layout->addWidget( _check_box_visible_trajectory_wrist, 0, 2 );
  plan_layout->addWidget( _slider_plan_current_index, 1, 0, 1, 3 );
  plan_widget->setLayout( plan_layout );
  plan_scroll_area->setWidget( plan_widget );

  QTabWidget * tab_widget = new QTabWidget( this );
  tab_widget->addTab( affordances_widget, QString( "affordances" ) ); 
  tab_widget->addTab( constraints_scroll_area, QString( "constraints" ) ); 
  tab_widget->addTab( plan_scroll_area, QString( "plan" ) ); 

  QGridLayout * widget_layout = new QGridLayout();
  widget_layout->addWidget( _widget_opengl_authoring );
  widget_layout->addWidget( _text_edit_info );
  widget_layout->addWidget( controls_group_box );
  widget_layout->addWidget( tab_widget );
  setLayout( widget_layout );

  connect( this, SIGNAL( affordance_collection_update( std::vector< affordance::AffordanceState >& ) ),
              _widget_opengl_authoring, SLOT( update_opengl_object_affordance_collection( std::vector< affordance::AffordanceState >& ) ) );
  connect( _slider_plan_current_index, SIGNAL( valueChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_current_index( int ) ) );
  connect( _check_box_visible_current_index, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_current_index( int ) ) );
  connect( _check_box_visible_trajectory, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_trajectory( int ) ) );
  connect( _check_box_visible_trajectory_wrist, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_trajectory_wrist( int ) ) );
  connect( this, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
  for( vector< Qt4_Widget_Constraint_Editor* >::iterator it = _constraint_editors.begin(); it != _constraint_editors.end(); it++ ){
    connect( *it, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
    connect( this, SIGNAL( time_min_update( double ) ), *it, SLOT( update_time_min( double ) ) );
    connect( this, SIGNAL( time_max_update( double ) ), *it, SLOT( update_time_max( double ) ) );
  }
  connect( _push_button_grab, SIGNAL( clicked() ), this, SLOT( _push_button_grab_pressed() ) );
  connect( _push_button_import, SIGNAL( clicked() ), this, SLOT( _push_button_import_pressed() ) );
  connect( _push_button_export, SIGNAL( clicked() ), this, SLOT( _push_button_export_pressed() ) );
  connect( _push_button_publish, SIGNAL( clicked() ), this, SLOT( _push_button_publish_pressed() ) );
  connect( _double_spin_box_end_time, SIGNAL( valueChanged( double ) ), this, SLOT( _double_spin_box_end_time_changed( double ) ) );
}

Qt4_Widget_Authoring::
~Qt4_Widget_Authoring() {

}

Qt4_Widget_Authoring::
Qt4_Widget_Authoring( const Qt4_Widget_Authoring& other ) : QWidget(),
                                                            _widget_opengl_authoring( new Qt4_Widget_OpenGL_Authoring( this ) ) {

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
update_affordance_collection( vector< AffordanceState >& affordanceCollection ){
  _affordance_collection_ghost = affordanceCollection;
  return;
}

void
Qt4_Widget_Authoring::
update_robot_plan( vector< State_GFE >& robotPlan ){
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
_push_button_grab_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] grab pressed" ) );
  _affordance_collection = _affordance_collection_ghost;
  _text_edit_affordance_collection->clear();
  for( vector< AffordanceState >::iterator it = _affordance_collection.begin(); it != _affordance_collection.end(); it++ ){
    _text_edit_affordance_collection->append( QString( "name: %1 uid: <%2,%3> xyz: (%4,%5,%6) rpy: (%7,%8,%9)" ).arg( QString::fromStdString( it->getName() ) ).arg( it->getGlobalUniqueId().first ).arg( it->getGlobalUniqueId().second ).arg( QString::number( it->getXYZ().x() ) ).arg( QString::number( it->getXYZ().y() ) ).arg( QString::number( it->getXYZ().z() ) ).arg( QString::number( it->getRPY().x() ) ).arg( QString::number( it->getRPY().y() ) ).arg( QString::number( it->getRPY().z() ) ) );
  }
  _state_gfe = _state_gfe_ghost;
  emit affordance_collection_update( _affordance_collection );
  emit state_gfe_update( _state_gfe );
  return;
}

void
Qt4_Widget_Authoring::
_push_button_import_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] import pressed" ) );

  QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                  "/home",
                                                  tr("ActioncSequence (*.bin)"));  
  if (fileName.isEmpty())
    return;

  //---read from file
  cout << "\n\n\n about to read file from disk" << endl;
  lcm::LogFile lFileReader(fileName.toUtf8().constData(), "r"); //'read'
  const lcm::LogEvent *eventFromFile = lFileReader.readNextEvent();
  drc::action_sequence_t action_sequence;
  action_sequence.decode(eventFromFile->data,0,eventFromFile->datalen);

  //---todo: now do something w/ the msg
  cout << "\nread file from disk." << endl;

  return;
}

void
Qt4_Widget_Authoring::
_push_button_export_pressed( void )
{
  emit info_update( QString( "[<b>OK</b>] export pressed" ) );
 
  //convert to lcm message
  action_sequence_t msg;
  _create_drc_action_sequence_t(msg);
  cout << "\n constructed message\n" << endl;
  cout << "\n\n encoded size = " << msg.getEncodedSize() << endl;

  //ask user for save file name
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                  "/home/untitled.bin",
                                                  tr("ActionSequence (*.bin)"));
  cout << "\n\n going to write to file = " << fileName.toUtf8().constData() << endl;
  
  if (fileName.isEmpty())
    return;

  //write to disk
  cout << "\n about to malloc w/ size = " << msg.getEncodedSize() << endl;
  void *buffer = malloc(msg.getEncodedSize());

  cout << "\n about to encode" << endl;
  msg.encode(buffer,0,msg.getEncodedSize());

  cout << "about to construct logEvent" << endl;

  lcm::LogEvent logEvent;
  logEvent.eventnum = 0;
  logEvent.timestamp = 0;
  logEvent.channel = "action_sequence_gui_io";
  logEvent.datalen = msg.getEncodedSize();
  logEvent.data = buffer;

  cout << "\n\nabout to write" << endl;

  lcm::LogFile lFileWriter(fileName.toUtf8().constData(), "w"); //'write'
  lFileWriter.writeEvent(&logEvent);


  cout << "\n wrote to file " << fileName.toUtf8().constData() << endl;

  return;
}

void
Qt4_Widget_Authoring::
_push_button_publish_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] publish pressed" ) );
  action_sequence_t msg;  
  _create_drc_action_sequence_t( msg );
  emit drc_action_sequence_t_publish( msg );      
  return;
}

void
Qt4_Widget_Authoring::
_double_spin_box_end_time_changed( double endTime ){
  emit time_max_update( _double_spin_box_end_time->value() );
  return;
} 

void
Qt4_Widget_Authoring::
_create_drc_action_sequence_t(action_sequence_t& msg)
{
  msg.num_contact_goals = 0;
  msg.robot_name = "atlas";
  //msg.q0.num_joints = 0;
  _state_gfe.to_lcm(&msg.q0);

  cout << "_constraints.size(): " << _constraints.size() << endl;
  for( vector< Constraint* >::iterator it = _constraints.begin(); it != _constraints.end(); it++ ){
    if( (*it) != NULL ){
      cout << "adding to drc action sequence" << endl;
      (*it)->add_to_drc_action_sequence_t(msg );
    }
  }
  cout << "utime: " << msg.utime << endl;
  cout << "robot_name: " << msg.robot_name << endl;
  cout << "num_contact_goals: " << msg.num_contact_goals << endl;
  for( vector< contact_goal_t >::iterator it = msg.contact_goals.begin(); it != msg.contact_goals.end(); it++ ){
    cout << "  object_1_name: " << it->object_1_name << endl;
    cout << "  object_1_contact_grp: " << it->object_1_contact_grp << endl;
    cout << "  object_2_name: " << it->object_2_name << endl;
    cout << "  object_2_contact_grp: " << it->object_2_contact_grp << endl;
    cout << "  lower_bound_completion_time: " << it->lower_bound_completion_time << endl;
    cout << "  upper_bound_completion_time: " << it->upper_bound_completion_time << endl;
    cout << "  contact_type: " << it->contact_type << endl;
    cout << "  x_offset: " << it->x_offset << endl;
    cout << "  y_offset: " << it->y_offset << endl;
    cout << "  z_offset: " << it->z_offset << endl;
    cout << "  x_relation: " << it->x_relation << endl;
    cout << "  y_relation: " << it->y_relation << endl;
    cout << "  z_relation: " << it->z_relation << endl;
  }
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Authoring& other ) {
    return out;
  }

}
