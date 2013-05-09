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
                                            _robot_model(),
                                            _affordance_collection(),
                                            _affordance_collection_ghost(){
  _robot_model.initString( Kinematics_Model_GFE::urdf_filename_to_xml_string( getModelsPath() + urdfFilename ) );

  _constraints.resize( numConstraints );
  for( unsigned int i = 0; i < numConstraints; i++ ){
    _constraints[ i ] = NULL;
    _constraint_editors.push_back( new Qt4_Widget_Constraint_Editor( _constraints[ i ], _robot_model, _affordance_collection, ( QString( "C%1" ).arg( QString::number( i ) ) ).toStdString(), this ) );
  }

  _text_edit_info->setFixedHeight( 75 );
  _double_spin_box_end_time->setSuffix( QString( " seconds" ) );

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
  connect( this, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
  for( vector< Qt4_Widget_Constraint_Editor* >::iterator it = _constraint_editors.begin(); it != _constraint_editors.end(); it++ ){
    connect( *it, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
/*
    connect( this, SIGNAL( affordance_collection_update( std::vector< affordance::AffordanceState >& ) ),
              *it, SLOT( update_affordance_collection( std::vector< affordance::AffordanceState >& ) ) );
*/
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
/*
  _text_edit_affordance_collection->clear();
  for( vector< AffordanceState >::iterator it = _affordance_collection.begin(); it != _affordance_collection.end(); it++ ){
    _text_edit_affordance_collection->append( QString( "name: %1 uid: <%2,%3> xyz: (%4,%5,%6) rpy: (%7,%8,%9)" ).arg( QString::fromStdString( it->getName() ) ).arg( it->getGlobalUniqueId().first ).arg( it->getGlobalUniqueId().second ).arg( QString::number( it->getXYZ().x() ) ).arg( QString::number( it->getXYZ().y() ) ).arg( QString::number( it->getXYZ().z() ) ).arg( QString::number( it->getRPY().x() ) ).arg( QString::number( it->getRPY().y() ) ).arg( QString::number( it->getRPY().z() ) ) );
  }
  emit affordance_collection_update( affordanceCollection );
*/
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
  emit affordance_collection_update( _affordance_collection );
  return;
}

void
Qt4_Widget_Authoring::
_push_button_import_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] import pressed" ) );

  QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                  "/home",
                                                  tr("ActioncSequence (*.bin)"));  
  //---read from file
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
  create_msg(msg);
 
  //ask user for save file name
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                  "/home/untitled.bin",
                                                  tr("ActionSequence (*.bin)"));
  //write to disk
  void *buffer = malloc(msg.getEncodedSize());
  msg.encode(buffer,0,msg.getEncodedSize());
  lcm::LogEvent logEvent;
  logEvent.eventnum = 0;
  logEvent.timestamp = 0;
  logEvent.channel = "action_sequence_gui_io";
  logEvent.datalen = msg.getEncodedSize();
  logEvent.data = buffer;
  lcm::LogFile lFileWriter(fileName.toUtf8().constData(), "w"); //'write'
  lFileWriter.writeEvent(&logEvent);
  
  return;
}

void 
Qt4_Widget_Authoring::
create_msg(action_sequence_t &action_sequence)
{
  action_sequence.num_contact_goals = 0;
  cout << "_constraints.size(): " << _constraints.size() << endl;
  for( vector< Constraint* >::iterator it = _constraints.begin(); it != _constraints.end(); it++ ){
    if( (*it) != NULL ){
      cout << "adding to drc action sequence" << endl;
      (*it)->add_to_drc_action_sequence_t( action_sequence );
    }
  }
  cout << "utime: " << action_sequence.utime << endl;
  cout << "robot_name: " << action_sequence.robot_name << endl;
  cout << "num_contact_goals: " << action_sequence.num_contact_goals << endl;
  for( vector< contact_goal_t >::iterator it = action_sequence.contact_goals.begin(); it != action_sequence.contact_goals.end(); it++ ){
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


void
Qt4_Widget_Authoring::
_push_button_publish_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] publish pressed" ) );
  action_sequence_t action_sequence;  
  create_msg(action_sequence);
             
  return;
}

void
Qt4_Widget_Authoring::
_double_spin_box_end_time_changed( double endTime ){
  emit time_max_update( _double_spin_box_end_time->value() );
  return;
} 

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Authoring& other ) {
    return out;
  }

}
