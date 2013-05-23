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
                                            _push_button_import( new QPushButton( QString( "import" ), this ) ),
                                            _push_button_export( new QPushButton( QString( "export" ), this ) ),
                                            _push_button_publish( new QPushButton( QString( "publish" ), this ) ),
                                            _text_edit_affordance_collection( new QTextEdit( "N/A", this ) ),
                                            _slider_plan_current_index( new QSlider( Qt::Horizontal, this ) ),
                                            _check_box_visible_current_index( new QCheckBox( "current index", this ) ),
                                            _check_box_visible_trajectory( new QCheckBox( "trajectory", this ) ),
                                            _check_box_visible_trajectory_wrist( new QCheckBox( "wrist trajectory", this ) ),
                                            _slider_current_time( new QLabel("frame 0") ),
                                            _robot_model(),
                                            _affordance_collection(),
                                            _affordance_collection_ghost(),
                                            _robot_plan(),
                                            _state_gfe_ghost(){
  _robot_model.initString( xmlString );

  _constraint_sequence.constraints().resize( numConstraints );
  for( unsigned int i = 0; i < numConstraints; i++ ){
    _constraint_sequence.constraints()[ i ] = NULL;
    _constraint_editors.push_back( new Qt4_Widget_Constraint_Editor( _constraint_sequence.constraints()[ i ], _robot_model, _affordance_collection, ( QString( "C%1" ).arg( QString::number( i ) ) ).toStdString(), this ) );
  }

  _text_edit_info->setFixedHeight( 75 );

  _check_box_visible_current_index->setCheckState( Qt::Checked );
  _check_box_visible_trajectory->setCheckState( Qt::Checked );
  _check_box_visible_trajectory_wrist->setCheckState( Qt::Unchecked );

  QGroupBox * controls_group_box = new QGroupBox( QString( "controls" ) );
  QHBoxLayout * controls_layout = new QHBoxLayout();
  controls_layout->addWidget( _push_button_grab );
  controls_layout->addWidget( _push_button_import );
  controls_layout->addWidget( _push_button_export );
  controls_layout->addWidget( _push_button_publish );
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
  plan_layout->addWidget( _check_box_visible_current_index, 0, 0 );
  plan_layout->addWidget( _check_box_visible_trajectory, 0, 1 );
  plan_layout->addWidget( _check_box_visible_trajectory_wrist, 0, 2 );
  plan_layout->addWidget( _slider_plan_current_index, 1, 0, 1, 3 );
  plan_layout->addWidget( _slider_current_time, 2, 0, 1, 3 );
  plan_widget->setLayout( plan_layout );
  plan_scroll_area->setWidget( plan_widget );

  QTabWidget * tab_widget = new QTabWidget( this );
  tab_widget->addTab( affordances_widget, QString( "affordances" ) ); 
  tab_widget->addTab( constraints_scroll_area, QString( "constraints" ) ); 
  tab_widget->addTab( plan_scroll_area, QString( "plan" ) ); 

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
  connect( this, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
  for( vector< Qt4_Widget_Constraint_Editor* >::iterator it = _constraint_editors.begin(); it != _constraint_editors.end(); it++ ){
    connect( *it, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
  }
  connect( _push_button_grab, SIGNAL( clicked() ), this, SLOT( _push_button_grab_pressed() ) );
  connect( _push_button_import, SIGNAL( clicked() ), this, SLOT( _push_button_import_pressed() ) );
  connect( _push_button_export, SIGNAL( clicked() ), this, SLOT( _push_button_export_pressed() ) );
  connect( _push_button_publish, SIGNAL( clicked() ), this, SLOT( _push_button_publish_pressed() ) );
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
update_affordance_collection( vector< AffordanceState >& affordanceCollection ){
  _affordance_collection_ghost = affordanceCollection;
  return;
}

void
Qt4_Widget_Authoring::
update_robot_plan( vector< State_GFE >& robotPlan ){
  emit info_update( QString( "[<b>OK</b> recieved plan" ) );
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
  _constraint_sequence.q0() = _state_gfe_ghost;
  cout << "grabbed state" << endl << _state_gfe_ghost << endl;
  emit affordance_collection_update( _affordance_collection );
  emit state_gfe_update( _constraint_sequence.q0() );
  return;
}

void
Qt4_Widget_Authoring::
_push_button_import_pressed( void ){
  QString filename = QFileDialog::getOpenFileName(this, tr("Load Constraint Sequence"),
                                                  "/home",
                                                  tr("ActioncSequence (*.bin)"));  
  if (filename.isEmpty()) {
    emit info_update( QString( "[<b>ERROR</b>] failed to import (filename empty)" ) );
    return;
  }

  _constraint_sequence.load( filename.toStdString(), _affordance_collection );
  emit info_update( QString( "[<b>OK</b>] imported constraint sequence to file %1" ).arg( filename ) );
  for( unsigned int i = 0; i < _constraint_editors.size(); i++ ){
    _constraint_editors[ i ]->update_constraint();
  } 
  return;
}

void
Qt4_Widget_Authoring::
_push_button_export_pressed( void )
{
  QString filename = QFileDialog::getSaveFileName(this, tr("Save Constraint Sequence"),
                                                  "/home/untitled.bin",
                                                  tr("ActionSequence (*.bin)"));
  
  if ( filename.isEmpty() ){
    emit info_update( QString( "[<b>ERROR</b>] failed to export (filename empty)" ) );
    return;
  }

  _constraint_sequence.save( filename.toStdString() );
  emit info_update( QString( "[<b>OK</b>] exported constraint sequence to file %1" ).arg( filename ) );

  return;
}

void
Qt4_Widget_Authoring::
_push_button_publish_pressed( void ){
  action_sequence_t msg; 
  _constraint_sequence.to_msg( msg ); 
  emit drc_action_sequence_t_publish( msg );     
  emit info_update( QString( "[<b>OK</b>] published constraint sequence as drc::action_sequence_t" ) ); 
  return;
}

void
Qt4_Widget_Authoring::
_slider_updated( int currentIndex ){
  if ( currentIndex < _robot_plan.size() ) {
    _slider_current_time->setText( QString( "frame %1" ).arg(     _robot_plan[currentIndex].time() ) );
  }
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Authoring& other ) {
    return out;
  }

}
