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
                                            _xmlString(xmlString),
                                            _widget_opengl_authoring( new Qt4_Widget_OpenGL_Authoring( xmlString, this ) ),
                                            _text_edit_info( new QTextEdit( "[<b>OK</b>] authoring widget started", this ) ),
                                            _push_button_grab( new QPushButton( QString( "grab" ), this ) ),
                                            _push_button_import( new QPushButton( QString( "import..." ), this ) ),
                                            _push_button_export( new QPushButton( QString( "export..." ), this ) ),
                                            _push_button_publish( new QPushButton( QString( "publish constraints" ), this ) ),
                                            _text_edit_affordance_collection( new QTextEdit( "N/A", this ) ),
                                            _slider_plan_current_index( new QSlider( Qt::Horizontal, this ) ),
                                            _label_plan_step_count( new QLabel("")),
                                            _check_box_visible_current_index( new QCheckBox( "current index", this ) ),
                                            _check_box_visible_trajectory( new QCheckBox( "trajectory", this ) ),
                                            _check_box_visible_trajectory_wrist( new QCheckBox( "wrist trajectory", this ) ),
                                            _check_box_visible_initial_state( new QCheckBox( "initial state", this ) ),
                                            _push_button_publish_plan( new QPushButton( QString( "publish plan" ), this ) ),
                                            _push_button_publish_reverse_plan( new QPushButton( QString( "reverse plan" ), this ) ),
                                            _double_spin_box_publish_start( new QDoubleSpinBox( this ) ),
                                            _double_spin_box_publish_range_label( new QLabel( "to" ) ),
                                            _double_spin_box_publish_end( new QDoubleSpinBox( this ) ),
                                            _progress_bar_planner( new QProgressBar( ) ),
                                            _label_planner_feedback( new QLabel( "No plan submitted.\n" ) ),
                                            _slider_current_time( new QLabel("frame 0") ),
                                            _robot_model(),
                                            _kinematics_model_gfe( xmlString ),
                                            _affordance_collection(),
                                            _affordance_collection_ghost(),
                                            _robot_plan(),
                                            _state_gfe_ghost(),
                                            _constraint_sequence( numConstraints ),
                                            _double_spin_box_insert_remove( new QDoubleSpinBox( this ) ),
                                            _push_button_insert_before( new QPushButton( QString( "insert before "), this )),
                                            _push_button_insert_after( new QPushButton( QString( "insert after "), this )),
                                            _push_button_remove_at( new QPushButton( QString( "delete at "), this )),
                                            _constraints_layout( new QGridLayout () ),
                                            _constraints_widget( new QWidget( this ) ),
                                            _constraints_layout_labels( new QWidget( this ) ),
                                            _constraints_scroll_area( new QScrollArea( this ) ),
                                            _constraint_ctr( 0 )
                                            {
  _robot_model.initString( xmlString );

  for( unsigned int i = 0; i < numConstraints; i++ ){
    _constraint_sequence.constraints()[ i ].id() = QString( "C%1" ).arg( _constraint_ctr ).toStdString();
    _constraint_sequence.constraints()[ i ].active() = false;
    _constraint_sequence.constraints()[ i ].visible() = false;
    _constraint_ctr++;
  }

  _push_button_publish->setEnabled( false );
  _push_button_publish->setToolTip("publish all constraints to a motion plan server over LCM");
  _push_button_grab->setToolTip("make all affordances and the robot currently in the scene available as \"child\" links for constraints");
  _push_button_import->setEnabled( false );
  _push_button_import->setToolTip("import all constraints from a file");
  _push_button_export->setEnabled( false );
  _push_button_export->setToolTip("export all constraints to a file");

  _progress_bar_planner->setMinimum( 0.0 );
  _progress_bar_planner->setMaximum( 0.0 );
  _progress_bar_planner->setFormat( QString("No plan submitted."));
  _progress_bar_planner->setTextVisible( true );

  _label_planner_feedback->setFrameStyle( QFrame::Panel | QFrame::Sunken );
  _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(0, 0, 0, 0); background-color: rgba(255, 0, 0, 0); color : black; }");
  _label_plan_step_count->setAlignment( Qt::AlignCenter );
  _text_edit_info->setFixedHeight( 75 );

  _text_edit_info->setTextInteractionFlags(Qt::TextSelectableByMouse);
  _text_edit_affordance_collection->setTextInteractionFlags(Qt::TextSelectableByMouse);

  _check_box_visible_current_index->setCheckState( Qt::Checked );
  _check_box_visible_trajectory->setCheckState( Qt::Checked );
  _check_box_visible_trajectory_wrist->setCheckState( Qt::Unchecked );
  _check_box_visible_initial_state->setCheckState( Qt::Checked );
  _push_button_publish_plan->setEnabled( false );
  _push_button_publish_reverse_plan->setEnabled( false );
  _push_button_publish_plan->setToolTip("broadcast current plan as a candidate plan");
  _push_button_publish_reverse_plan->setToolTip("broadcast current plan, time-reversed, as a candidate plan");
  _double_spin_box_publish_start->setEnabled( false );
  _double_spin_box_publish_end->setEnabled( false );

  QGroupBox * controls_group_box = new QGroupBox( QString( "controls" ) );
  QGridLayout * controls_layout = new QGridLayout();
  controls_layout->addWidget( _push_button_grab, 0, 0 );
  controls_layout->addWidget( _push_button_import, 0, 1 );
  controls_layout->addWidget( _push_button_export, 0, 2 );
  controls_layout->addWidget( _push_button_publish, 0, 3 );
  controls_group_box->setLayout( controls_layout );
 
  QWidget * affordances_widget = new QWidget( this );
  QGridLayout * affordances_layout = new QGridLayout();
  affordances_layout->addWidget( _text_edit_affordance_collection );
  affordances_widget->setLayout( affordances_layout );
 
  QWidget * constraints_meta_widget  = new QWidget( this );
  QGridLayout * constraints_meta_layout = new QGridLayout();
  _constraints_scroll_area->setFrameStyle( QFrame::NoFrame );

  // Insert / remove constraint controls
  _double_spin_box_insert_remove->setDecimals(0);
  constraints_meta_layout->addWidget(_double_spin_box_insert_remove, 0, 0);
  constraints_meta_layout->addWidget(_push_button_insert_before, 1, 0);
  constraints_meta_layout->addWidget(_push_button_insert_after, 2, 0);
  constraints_meta_layout->addWidget(_push_button_remove_at, 3, 0);

  // prepare labels for constraint editor bars
  char stylesheet[] = "QLabel { border: 2px solid rgba(0, 0, 0, 150); background-color: rgba(255, 0, 0, 0); color : black; }";
  QHBoxLayout * constraints_layout_labels_layout = new QHBoxLayout();
  QWidget * tmp =  new QLabel("");
  //tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 35 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("En") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 40 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("Vis") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 40 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 50 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 50 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("Start") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 70 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("End") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 70 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("Comment") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 200 ); constraints_layout_labels_layout->addWidget( tmp );
  tmp =  new QLabel( QString("") );
  tmp->setStyleSheet(stylesheet);
  tmp->setFixedWidth( 1024 ); constraints_layout_labels_layout->addWidget( tmp );
  // layout for now contains just labels; we'll sync up and get editors for
  //  existing constraints in just a bit
  _constraints_layout_labels->setLayout(constraints_layout_labels_layout);
  _constraints_layout->addWidget(_constraints_layout_labels, 0, 1);
  _constraints_widget->setLayout( _constraints_layout );
  _constraints_scroll_area->setWidget( _constraints_widget ); 
  constraints_meta_layout->addWidget(_constraints_scroll_area, 0, 1, 8, 1);
  constraints_meta_widget->setLayout(constraints_meta_layout);

  _slider_plan_current_index->setTickPosition(QSlider::TicksBelow);
  
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
  plan_layout->addWidget( _label_plan_step_count, 1, 4, 1, 1);
  plan_layout->addWidget( _slider_current_time, 2, 0, 1, 5 );
  plan_layout->addWidget( _progress_bar_planner, 3, 0, 1, 2);
  plan_layout->addWidget( _push_button_publish_plan, 3, 2);
  plan_layout->addWidget( _push_button_publish_reverse_plan, 3, 3);
  plan_layout->addWidget( _label_planner_feedback, 4, 0, 1, 4);
  plan_layout->addWidget( _double_spin_box_publish_start, 2, 2, 1, 1);
  plan_layout->addWidget( _double_spin_box_publish_range_label, 2, 3, 1, 1);
  plan_layout->addWidget( _double_spin_box_publish_end, 2, 4, 1, 1);
  plan_widget->setLayout( plan_layout );

  plan_scroll_area->setWidget( plan_widget );

  QTabWidget * tab_widget = new QTabWidget( this );
  tab_widget->addTab( constraints_meta_widget, QString( "constraints" ) ); 
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

  _refresh_constraint_editors();

  connect( this, SIGNAL( affordance_collection_update( std::vector< affordance::AffordanceState >& ) ),
              _widget_opengl_authoring, SLOT( update_opengl_object_affordance_collection( std::vector< affordance::AffordanceState >& ) ) );
  connect( this, SIGNAL( state_gfe_update( state::State_GFE& ) ), _widget_opengl_authoring, SLOT( update_opengl_object_gfe( state::State_GFE& ) ) );
  connect( _slider_plan_current_index, SIGNAL( valueChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_current_index( int ) ) );
  connect( _slider_plan_current_index, SIGNAL( valueChanged( int ) ), this, SLOT( _slider_updated( int ) ) );
  connect( _check_box_visible_current_index, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_current_index( int ) ) );
  connect( _check_box_visible_trajectory, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_trajectory( int ) ) );
  connect( _check_box_visible_trajectory_wrist, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_trajectory_wrist( int ) ) );
  connect( _check_box_visible_initial_state, SIGNAL( stateChanged( int ) ), _widget_opengl_authoring, SLOT( update_opengl_object_robot_plan_visible_initial_state( int ) ) );
  connect( _push_button_publish_plan, SIGNAL( clicked() ), this, SLOT( _push_button_publish_plan_pressed() ) );
  connect( _push_button_publish_reverse_plan, SIGNAL( clicked() ), this, SLOT( _push_button_publish_reverse_plan_pressed() ) );

  connect( this, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
  connect( _widget_opengl_authoring, SIGNAL( publish_constraints( float ) ), this, SLOT( publish_constraints( float ) ) );

  connect( _push_button_grab, SIGNAL( clicked() ), this, SLOT( _push_button_grab_pressed() ) );
  connect( _push_button_import, SIGNAL( clicked() ), this, SLOT( _push_button_import_pressed() ) );
  connect( _push_button_export, SIGNAL( clicked() ), this, SLOT( _push_button_export_pressed() ) );
  connect( _push_button_publish, SIGNAL( clicked() ), this, SLOT( _push_button_publish_pressed() ) );

  connect( _push_button_insert_before, SIGNAL( clicked() ), this, SLOT( _push_button_insert_before_pressed() ) );
  connect( _push_button_insert_after, SIGNAL( clicked() ), this, SLOT( _push_button_insert_after_pressed() ) );
  connect( _push_button_remove_at, SIGNAL( clicked() ), this, SLOT( _push_button_remove_at_pressed() ) );
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
update_constraint( const Constraint_Task_Space_Region& constraint ){
  for (int i=0; i<_constraint_sequence.constraints().size(); i++){
    if (_constraint_sequence.constraints()[i].id() == constraint.id()){
      _constraint_sequence.constraints()[i] = constraint;
      _widget_opengl_authoring->update_opengl_object_constraint_sequence( _constraint_sequence );
      return;
    }
  }
  cout << "update_constraint couldn't find constraint " << constraint.id() << endl;
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
  int old_pos = _slider_plan_current_index->value();
  long long int old_tpos = 0.0;
  if (old_pos < _robot_plan.size())
    old_tpos = _robot_plan[old_pos].time();

  _robot_plan = robotPlan;
  _slider_plan_current_index->setRange( 0, ( robotPlan.size() - 1 ) );

  // try to time point that we used to be on, otherwise jump to end
  int new_pos = _robot_plan.size()-1;
  for (int i=0; i<_robot_plan.size(); i++){
    if (_robot_plan[i].time() == old_tpos){
      new_pos = i;
      break;
    }
  }
  _slider_plan_current_index->setSliderPosition( new_pos );

  // update plan slice ctr
  char tmp[100];
  sprintf(tmp, "%u time points", (unsigned int)_robot_plan.size());
  _label_plan_step_count->setText(tmp);

  return;
}

void
Qt4_Widget_Authoring::
insert_robot_plan( State_GFE& robotPlanSlice ){
  emit info_update( QString( "[<b>OK</b>] received plan slice" ) );
  // figure out where it fits into current plan
  bool inserted = false;
  int fit_index = 0;
  for (int i=0; i<_robot_plan.size(); i++){
    if (_robot_plan[i].time() == robotPlanSlice.time()){
      _robot_plan[i] = robotPlanSlice;
      inserted = true;
      fit_index = i;
      break;
    }
    if (_robot_plan[i].time() > robotPlanSlice.time()){
      // it fits in the middle somewhere but time slice is novel
      _robot_plan.insert(_robot_plan.begin()+i, robotPlanSlice);
      inserted = true;
      fit_index = i;
      break;
    }
  }
  if (!inserted){
    // tack on to end
    _robot_plan.push_back(robotPlanSlice);
    fit_index = _robot_plan.size()-1;
  }

  _widget_opengl_authoring->update_opengl_object_robot_plan(_robot_plan);
  _slider_plan_current_index->setRange( 0, ( _robot_plan.size() - 1 ) );
  _slider_plan_current_index->setSliderPosition( fit_index );
  _slider_updated( fit_index );

  // update plan slice ctr
  char tmp[100];
  sprintf(tmp, "%u time points", (unsigned int)_robot_plan.size());
  _label_plan_step_count->setText(tmp);
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
aas_got_status_msg( bool server_ready_msg, float last_time_solved,  float total_time_to_solve,
      bool solving_highres, bool plan_is_good, bool plan_is_warn ){
  char tmp[100];

  _push_button_publish->setEnabled(true);
  // if it's a "server is ready" message, that's everything
  if (server_ready_msg){
    return;
  }

  // otherwise, update more complex status.
  _progress_bar_planner->setMinimum( 0.0 );
  _progress_bar_planner->setMaximum( 100.0 );
  _progress_bar_planner->setValue( (int)(100.0 * last_time_solved / total_time_to_solve) );

  // first -- if plan failed, better catch it!
  if (!plan_is_good){
      _progress_bar_planner->setFormat( QString("Planning failed!") );
      if (solving_highres){
        sprintf(tmp, "Constraints impossible in highres at time %fs", last_time_solved);
      } else {
        sprintf(tmp, "Constraints impossible in low res at time %fs", last_time_solved);
      }
      _label_planner_feedback->setText(tmp);
      _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(255, 0, 0, 150); background-color: rgba(255, 0, 0, 0); color : black; }");
  
      // go highlight all constraints active during that time window.
      vector< string > all_in_error;
      for (int i=0; i < _constraint_sequence.constraints().size(); i++){
        if (_constraint_sequence.constraints()[i].start() <= last_time_solved &&
           _constraint_sequence.constraints()[i].end() >= last_time_solved)
          all_in_error.push_back(_constraint_sequence.constraints()[i].id());
      }
      _widget_opengl_authoring->highlight_constraints(all_in_error, HIGHLIGHT_RED);

  } else { // if not failed...
    // if this is the last time in either
    if (last_time_solved == total_time_to_solve && solving_highres){
        _progress_bar_planner->setFormat( QString("Fully planned!") );
        _push_button_publish_plan->setEnabled( true );
        _push_button_publish_reverse_plan->setEnabled( true );
        // and enable range buttons
        _double_spin_box_publish_start->setEnabled( true );
        _double_spin_box_publish_end->setEnabled( true );
        _double_spin_box_publish_start->setMinimum( 0.0 );
        _double_spin_box_publish_end->setMinimum( 0.0 );
        _double_spin_box_publish_start->setMaximum( total_time_to_solve );
        _double_spin_box_publish_end->setMaximum( total_time_to_solve );
        _double_spin_box_publish_start->setValue( 0.0 );
        _double_spin_box_publish_end->setValue( total_time_to_solve );
        if (plan_is_warn){
          _label_planner_feedback->setText("Plan ready for publish, with warnings.");
          _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(255, 255, 0, 150); background-color: rgba(255, 0, 0, 0); color : black; }");
        } else {
          _label_planner_feedback->setText("Plan ready for publish, no warnings.");
          _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(0, 255, 0, 150); background-color: rgba(255, 0, 0, 0); color : black; }");
        }
    } else {
      if (solving_highres){
        sprintf(tmp, "Highres: %%p%%: %fs", last_time_solved);
      } else {
        sprintf(tmp, "Low res: %%p%%: %fs", last_time_solved);
        // if this is the last step of the low res then set label that we're waiting on highres
        if (last_time_solved == total_time_to_solve)
          _label_planner_feedback->setText("Waiting for fine plan from server...");
      }
      _progress_bar_planner->setFormat( QString(tmp) );
    }
  }
  return;
}

void 
Qt4_Widget_Authoring::
publish_constraints( float ik_time_of_interest ){
  // publish a single-shot at just our current time
  action_sequence_t msg; 
  _constraint_sequence.to_msg( msg, _affordance_collection, ik_time_of_interest ); 
  Constraint_Sequence::print_msg( msg );
  msg.ik_time = ik_time_of_interest;
  if (ik_time_of_interest != -1 && _robot_plan.size() > 0){
    _robot_plan[_slider_plan_current_index->value()].to_lcm(&msg.q0);
  }

  emit drc_action_sequence_t_oneshot_publish( msg );  
  _push_button_publish_plan->setEnabled( false );
  _push_button_publish_reverse_plan->setEnabled( false );
  emit info_update( QString( "[<b>OK</b>] published one-shot constraint sequence as drc::action_sequence_t" ) ); 
  
  // also get to work on the full plan
  _push_button_publish_pressed();

  return;

}

void
Qt4_Widget_Authoring::
_push_button_grab_pressed( void ){
  emit info_update( QString( "[<b>OK</b>] grab pressed" ) );
  emit info_update( QString( "[<b>OK</b>] waiting on ready message from server before publishing is enabled" ) );
  _push_button_import->setEnabled( true );
  _push_button_export->setEnabled( true );
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
  _widget_opengl_authoring->update_opengl_object_constraint_sequence(_constraint_sequence);
  _refresh_constraint_editors();
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
  _push_button_publish_plan->setEnabled( false );
  _push_button_publish_reverse_plan->setEnabled( false );
  _label_planner_feedback->setText("Waiting for rough plan from server...");
  _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(0, 0, 0, 0); background-color: rgba(255, 0, 0, 0); color : black; }");
  _widget_opengl_authoring->highlight_constraints(vector<string>(), HIGHLIGHT_RED);
  emit info_update( QString( "[<b>OK</b>] published constraint sequence as drc::action_sequence_t" ) ); 
  return;
}

void
Qt4_Widget_Authoring::
_push_button_publish_plan_pressed( void ){
  robot_plan_w_keyframes_t msg;
  robot_state_t tmp;
  msg.robot_name = "atlas";
  msg.utime = 0;
  long start_time = (long) (1000000.0 * _double_spin_box_publish_start->value());
  long end_time = (long) (1000000.0 * _double_spin_box_publish_end->value());
  if (end_time <= start_time){
    emit info_update( QString( "[<b>ERROR</b>] not publishing robot plan: invalid time range" ) ); 
    return;
  }

  for ( unsigned int i = 0; i < _robot_plan.size(); i++){
    if (_robot_plan[i].time() >= start_time && _robot_plan[i].time() <= end_time){
      _robot_plan[i].to_lcm_minimal( &tmp );
      //tmp.robot_name = "atlas";
      tmp.utime -= start_time;
      msg.plan.push_back(tmp);
    }
  }
  msg.num_states = msg.plan.size();
  // and grasps (none)
  msg.num_grasp_transitions = 0;
  // and matlab payload (none)
  msg.num_bytes = 0;
  // and key/breakpoints, of which we use none
  msg.num_keyframes = 0;
  msg.num_breakpoints = 0;
  for (int i=0; i<msg.num_states; i++){
    msg.is_keyframe.push_back(false);
    msg.is_breakpoint.push_back(false);
  }

  emit robot_plan_w_keyframes_t_publish( msg ); 
  _label_planner_feedback->setText("No plan submitted.\n");
  _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(0, 0, 0, 0); background-color: rgba(255, 0, 0, 0); color : black; }");
  emit info_update( QString( "[<b>OK</b>] published robot plan as drc::robot_plan_w_keyframes_t" ) ); 
  return;
}

void
Qt4_Widget_Authoring::
_push_button_publish_reverse_plan_pressed( void ){
  robot_plan_w_keyframes_t msg;
  robot_state_t tmp;
  msg.robot_name = "atlas";
  msg.utime = 0;

  long start_time = (long) (1000000.0 * _double_spin_box_publish_start->value());
  long end_time = (long) (1000000.0 * _double_spin_box_publish_end->value());
  if (end_time <= start_time){
    emit info_update( QString( "[<b>ERROR</b>] not publishing robot plan: invalid time range" ) ); 
    return;
  }

  long last_state_time;
  bool found_starting_point = false;
  for ( int i = _robot_plan.size() - 1; i >= 0; i--){
    _robot_plan[i].to_lcm_minimal( &tmp );
    if (found_starting_point){
      //tmp.robot_name = "atlas";
      if (tmp.utime < start_time) break;
      tmp.utime = last_state_time - tmp.utime;
      msg.plan.push_back(tmp); 
    } else {
      if (tmp.utime <= end_time){
        last_state_time = tmp.utime;
        found_starting_point = true;
        tmp.utime = end_time - tmp.utime;
        msg.plan.push_back(tmp); 
      }
    }
  }
  msg.num_states = msg.plan.size();
  // and grasps (none)
  msg.num_grasp_transitions = 0;
  // and matlab payload (none)
  msg.num_bytes = 0;
  // and key/breakpoints, of which we use none
  msg.num_keyframes = 0;
  msg.num_breakpoints = 0;
  for (int i=0; i<msg.num_states; i++){
    msg.is_keyframe.push_back(false);
    msg.is_breakpoint.push_back(false);
  }

  emit robot_plan_w_keyframes_t_publish( msg ); 
  _label_planner_feedback->setText("No plan submitted.\n");
  _label_planner_feedback->setStyleSheet("QLabel { border: 2px solid rgba(0, 0, 0, 0); background-color: rgba(255, 0, 0, 0); color : black; }");
  emit info_update( QString( "[<b>OK</b>] published reversed robot plan as drc::robot_plan_w_keyframes_t" ) ); 
  return;
}

void 
Qt4_Widget_Authoring::
_push_button_insert_before_pressed( void ){
  _add_new_constraint( ((int) _double_spin_box_insert_remove->value()) );
  return;
}
void 
Qt4_Widget_Authoring::
_push_button_insert_after_pressed( void ){
  _add_new_constraint( ((int) _double_spin_box_insert_remove->value()) + 1 );
  return;
}
void 
Qt4_Widget_Authoring::
_add_new_constraint( int constraint_num ){
  if (constraint_num < 0 || constraint_num > _constraint_sequence.constraints().size())
    return;

  //otherwise, this is a valid call, so modify constraint list appropriately
  Constraint_Task_Space_Region new_ctsr = Constraint_Task_Space_Region(
              QString( "C%1" ).arg( _constraint_ctr ).toStdString());
  new_ctsr.active() = false;
  new_ctsr.visible() = false;
  _constraint_ctr++;
  _constraint_sequence.constraints().insert(_constraint_sequence.constraints().begin()+constraint_num, new_ctsr);

  // selections are likely to be messed up so disable
  _widget_opengl_authoring->bind_axes_to_constraint(NULL, true);

  //do the same for the opengl constraints
  _widget_opengl_authoring->update_opengl_object_constraint_sequence(_constraint_sequence);

  //do the same for the constraint editor list
  _refresh_constraint_editors();
  return;
}

void 
Qt4_Widget_Authoring::
_push_button_remove_at_pressed( void ){
  int constraint_num = (int) _double_spin_box_insert_remove->value();

  if (constraint_num < 0 || constraint_num >= _constraint_sequence.constraints().size())
    return;

  // valid call; modify constraint list to remove this constraint
  _constraint_sequence.constraints().erase(_constraint_sequence.constraints().begin()+constraint_num);

  // selections are likely to be messed up so disable
  _widget_opengl_authoring->bind_axes_to_constraint(NULL, true);

  //do the same for the opengl constraints
  _widget_opengl_authoring->update_opengl_object_constraint_sequence(_constraint_sequence);

  //do the same for the constraint editor list
  _refresh_constraint_editors();
  return;
}

// Regenerates constraint editor list to conform to current
// _constraint_sequence
void
Qt4_Widget_Authoring::
_refresh_constraint_editors( void ){
  // clear out old editors if any
  for (int i=0; i<_constraint_editors.size(); i++){
    _constraints_layout->removeWidget(_constraint_editors[i]);
    delete _constraint_editors[i];
  }
  _constraint_editors.erase(_constraint_editors.begin(), _constraint_editors.end());
  delete _constraints_layout;

  // spawn a bunch of new ones
  for (int i=0; i<_constraint_sequence.constraints().size(); i++){
    _constraint_editors.push_back( new Qt4_Widget_Constraint_Editor( _constraint_sequence.constraints()[ i ], _robot_model, _affordance_collection, _xmlString, i, this ) );
  }

  // hook them up to appropriate handlers
  for( vector< Qt4_Widget_Constraint_Editor* >::iterator it = _constraint_editors.begin(); it != _constraint_editors.end(); it++ ){
    connect( *it, SIGNAL( info_update( const QString& ) ), this, SLOT( update_info( const QString& ) ) );
    connect( *it, SIGNAL( constraint_update( const Constraint_Task_Space_Region& ) ), this, SLOT( update_constraint( const Constraint_Task_Space_Region& ) ) );
    connect( *it, SIGNAL( constraint_highlight ( const QString&, highlight_class_t, bool ) ), _widget_opengl_authoring, SLOT( highlight_constraint( const QString&, highlight_class_t, bool ) ) );
    connect( *it, SIGNAL( child_highlight ( const QString&, const QString&, bool ) ), _widget_opengl_authoring, SLOT( highlight_child( const QString&, const QString&, bool ) ) );
    connect( *it, SIGNAL( bind_axes_to_constraint ( Constraint_Task_Space_Region *, bool ) ), _widget_opengl_authoring, SLOT( bind_axes_to_constraint( Constraint_Task_Space_Region *, bool ) ) );
    connect( *it, SIGNAL( unbind_axes_from_constraint ( Constraint_Task_Space_Region * ) ), _widget_opengl_authoring, SLOT( unbind_axes_from_constraint( Constraint_Task_Space_Region * ) ) );
    connect( _widget_opengl_authoring, SIGNAL( update_constraint ( const Constraint_Task_Space_Region& ) ), this, SLOT( update_constraint( const Constraint_Task_Space_Region& ) ) );
    connect( _widget_opengl_authoring, SIGNAL( select_constraint( const QString&, select_class_t) ), *it, SLOT( select_constraint( const QString&, select_class_t) ) );
  }

  // re-add editor bars to layout
  _constraints_layout = new QGridLayout();
  _constraints_layout->addWidget( _constraints_layout_labels, 0, 1);
  for( int i=0; i<_constraint_editors.size(); i++ ){
    _constraints_layout->addWidget( _constraint_editors[i], i+1, 1 );
  }
  _constraints_widget = _constraints_scroll_area->takeWidget( );
  // enforce 60 pixels per row
  _constraints_widget->setMaximumHeight((_constraint_editors.size()+1) * 60);
  _constraints_widget->setLayout( _constraints_layout );
  _constraints_widget->show();
  _constraints_scroll_area->setWidget( _constraints_widget ); 

  // While we're here, ensure that our _constraint_ctr is greater than any existing
  //  current constraint.
  for (int i=0; i<_constraint_sequence.constraints().size(); i++){
    int cnt = atoi(&_constraint_sequence.constraints()[i].id().c_str()[1]);
    if (cnt >= _constraint_ctr)
      _constraint_ctr = cnt+1;
  }

  // and set limits on constraint spinbox
  _double_spin_box_insert_remove->setMinimum(0);
  _double_spin_box_insert_remove->setMaximum(_constraint_editors.size()-1);
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
