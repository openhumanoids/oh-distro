#include <QtGui/QHBoxLayout>

#include "authoring/qt4_widget_constraint_task_space_region_editor.h"
#include "authoring/qt4_widget_constraint_editor.h"

using namespace std;
using namespace boost;
using namespace urdf;
using namespace affordance;
using namespace authoring;

Qt4_Widget_Constraint_Editor::
Qt4_Widget_Constraint_Editor( Constraint *& constraint,
                              Model& robotModel,
                              vector< AffordanceState >& affordanceCollection,
                              const std::string& id,
                              QWidget * parent ) : QWidget( parent ),
                                                    _constraint( constraint ),
                                                    _robot_model( robotModel ),
                                                    _robot_affordances(),
                                                    _object_affordances( affordanceCollection ),
                                                    _id( id ),
                                                    _time_min( 0.0 ),
                                                    _time_max( 0.0 ),
                                                    _label_id( new QLabel( QString::fromStdString( id ), this ) ),
                                                    _check_box_active( new QCheckBox( this ) ),
                                                    _combo_box_type( new QComboBox( this ) ),
                                                    _push_button_edit( new QPushButton( QString( "edit" ), this ) ),
                                                    _widget_double_slider( new Qt4_Widget_Double_Slider() ),
                                                    _label_time_start( new QLabel( "N/A", this ) ),
                                                    _label_time_end( new QLabel( "N/A", this ) ),
                                                    _constraint_editor_popup( NULL ) {
  vector< shared_ptr< Link > > links;
  _robot_model.getLinks( links );
  for( vector< shared_ptr< Link > >::iterator it1 = links.begin(); it1 != links.end(); it1++ ){
    for( map< string, shared_ptr< vector< shared_ptr< Collision > > > >::iterator it2 = (*it1)->collision_groups.begin(); it2 != (*it1)->collision_groups.end(); it2++ ){
      _robot_affordances.push_back( pair< shared_ptr< Link >, std::string >( (*it1), it2->first ) );
    }
  }

  for( unsigned int i = 0; i < NUM_CONSTRAINT_TYPES; i++ ){
    _combo_box_type->addItem( QString::fromStdString( Constraint::constraint_type_t_to_std_string( ( constraint_type_t )( i ) ) ) );
  }

  _label_id->setFixedWidth( 25 );
  _check_box_active->setFixedWidth( 25 );
  _combo_box_type->setFixedWidth( 75 );
  _push_button_edit->setFixedWidth( 50 );

  QHBoxLayout * widget_layout = new QHBoxLayout();
  widget_layout->addWidget( _check_box_active );
  widget_layout->addWidget( _label_id );
  widget_layout->addWidget( _combo_box_type );
  widget_layout->addWidget( _push_button_edit );
  widget_layout->addWidget( _widget_double_slider );
  widget_layout->addWidget( _label_time_start );
  widget_layout->addWidget( _label_time_end );
  setLayout( widget_layout );

  connect( _widget_double_slider, SIGNAL( start_update( double ) ), this, SLOT( _update_start( double ) ) );
  connect( _widget_double_slider, SIGNAL( end_update( double ) ), this, SLOT( _update_end( double ) ) );
  connect( _check_box_active, SIGNAL( stateChanged( int ) ), this, SLOT( _check_box_active_changed( int ) ) );
  connect( _combo_box_type, SIGNAL( currentIndexChanged( int ) ), this, SLOT( _combo_box_type_changed( int ) ) );
  connect( _push_button_edit, SIGNAL( clicked() ), this, SLOT( _push_button_edit_pressed() ) );

  if( constraint != NULL ){
    _combo_box_type->setEnabled( constraint->active() );
    _push_button_edit->setEnabled( constraint->active() );
    _widget_double_slider->setEnabled( constraint->active() );
    _label_time_start->setEnabled( constraint->active() );
    _label_time_end->setEnabled( constraint->active() );
  } else {
    _combo_box_type->setEnabled( false );
    _push_button_edit->setEnabled( false );
    _widget_double_slider->setEnabled( false );
    _label_time_start->setEnabled( false );
    _label_time_end->setEnabled( false );
  }
}

Qt4_Widget_Constraint_Editor::
~Qt4_Widget_Constraint_Editor() {

}

Qt4_Widget_Constraint_Editor::
Qt4_Widget_Constraint_Editor( const Qt4_Widget_Constraint_Editor& other ) : QWidget(),
                                                                            _constraint( other._constraint ),
                                                                            _robot_model( other._robot_model ),
                                                                            _object_affordances( other._object_affordances ) {

}

Qt4_Widget_Constraint_Editor&
Qt4_Widget_Constraint_Editor::
operator=( const Qt4_Widget_Constraint_Editor& other ) {

  return (*this);
}
/*
void
Qt4_Widget_Constraint_Editor::
update_object_affordances( vector< AffordanceState >& affordanceCollection ){
  _object_affordances = affordanceCollection;
  return;
}
*/
void
Qt4_Widget_Constraint_Editor::
update_time_min( double timeMin ){
  _time_min = timeMin;
  _update_start( _widget_double_slider->start()->x() );
  _update_end( _widget_double_slider->end()->x() );
  return;
}

void
Qt4_Widget_Constraint_Editor::
update_time_max( double timeMax ){
  _time_max = timeMax;
  _update_start( _widget_double_slider->start()->x() );
  _update_end( _widget_double_slider->end()->x() );
  return;
}

void
Qt4_Widget_Constraint_Editor::
_update_start( double index ){
  double start = _time_min + ( double )( index - _widget_double_slider->min() ) / ( double )( _widget_double_slider->max() - _widget_double_slider->min() ) * ( double )( _time_max - _time_min );
  if( _constraint != NULL ){
    _constraint->start() = start;
    _label_time_start->setText( QString::number( _constraint->start(), 'f', 2 ) );
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
_update_end( double index ){
  double end = _time_min + ( double )( index - _widget_double_slider->min() ) / ( double )( _widget_double_slider->max() - _widget_double_slider->min() ) * ( double )( _time_max - _time_min );
  if( _constraint != NULL ){
    _constraint->end() = end;
    _label_time_end->setText( QString::number( _constraint->end(), 'f', 2 ) );
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
_check_box_active_changed( int state ){
  switch( _check_box_active->checkState() ){
  case ( Qt::Unchecked ):
  case ( Qt::PartiallyChecked ):
    _combo_box_type->setEnabled( false );
    _push_button_edit->setEnabled( false );
    _widget_double_slider->setEnabled( false );
    _label_time_start->setEnabled( false );
    _label_time_end->setEnabled( false );
    _combo_box_type->setCurrentIndex( CONSTRAINT_UNKNOWN_TYPE );
    if( _constraint_editor_popup != NULL ){
      _constraint_editor_popup->close(); 
      delete _constraint_editor_popup;
      _constraint_editor_popup = NULL;
    }
    if( _constraint != NULL ){
      emit info_update( QString( "[<b>OK</b>] deactivating constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      delete _constraint;
      _constraint = NULL; 
    } else {
      emit info_update( QString( "[<b>OK</b>] deactivating constraint %1" ).arg( QString::fromStdString( _id ) ) );
    }
    break;
  case ( Qt::Checked ):
    _combo_box_type->setEnabled( true );
    if( _constraint != NULL ){
      _constraint->active() = true;
      emit info_update( QString( "[<b>OK</b>] activating constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
    } else {
      emit info_update( QString( "[<b>OK</b>] activating constraint %1" ).arg( QString::fromStdString( _id ) ) );
    }
    break;
  default:
    break;
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
_combo_box_type_changed( int index ){
  if( _constraint != NULL ){
    delete _constraint;
    _constraint = NULL;
    emit info_update( QString( "[<b>OK</b>] deleted constraint %1" ).arg( QString::fromStdString( _id ) ) );
    switch( _combo_box_type->currentIndex() ){
    case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
      _push_button_edit->setEnabled( true );
      _widget_double_slider->setEnabled( true );
      _label_time_start->setEnabled( true );
      _label_time_end->setEnabled( true );
      _constraint = new Constraint_Task_Space_Region( _id );
      emit info_update( QString( "[<b>OK</b>] instatiated new point-to-point constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_CONFIGURATION_TYPE ):
      _push_button_edit->setEnabled( true );
      _widget_double_slider->setEnabled( true );
      _label_time_start->setEnabled( true );
      _label_time_end->setEnabled( true );
      break;
    case ( CONSTRAINT_UNKNOWN_TYPE ):
    default:
      _push_button_edit->setEnabled( false );
      _widget_double_slider->setEnabled( false );
      _label_time_start->setEnabled( false );
      _label_time_end->setEnabled( false );
      break;
    }
  } else {
    switch( _combo_box_type->currentIndex() ){
    case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
      _push_button_edit->setEnabled( true );
      _widget_double_slider->setEnabled( true );
      _label_time_start->setEnabled( true );
      _label_time_end->setEnabled( true );
      _constraint = new Constraint_Task_Space_Region( _id );
      if( !_robot_affordances.empty() ){
        dynamic_cast< Constraint_Task_Space_Region* >( _constraint )->parent().first = _robot_affordances.begin()->first;
        dynamic_cast< Constraint_Task_Space_Region* >( _constraint )->parent().second = _robot_affordances.begin()->second;
      } 
      if( !_object_affordances.empty() ){
        dynamic_cast< Constraint_Task_Space_Region* >( _constraint )->child() = &(*_object_affordances.begin());
      } 
      emit info_update( QString( "[<b>OK</b>] instatiated new point-to-point constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_CONFIGURATION_TYPE ):
      _push_button_edit->setEnabled( true );
      _widget_double_slider->setEnabled( true );
      _label_time_start->setEnabled( true );
      _label_time_end->setEnabled( true );
      break;  
    case ( CONSTRAINT_UNKNOWN_TYPE ):
    default:
      _push_button_edit->setEnabled( false );
      _widget_double_slider->setEnabled( false );
      _label_time_start->setEnabled( false );
      _label_time_end->setEnabled( false );
      break;
    }
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
_push_button_edit_pressed( void ){
  if( _constraint != NULL ){
    switch( _constraint->type() ){
    case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
      _constraint_editor_popup = new Qt4_Widget_Constraint_Task_Space_Region_Editor( dynamic_cast< Constraint_Task_Space_Region* >( _constraint ), _robot_model, _object_affordances, this );
      _constraint_editor_popup->show();
      emit info_update( QString( "[<b>OK</b>] launching editor for constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_CONFIGURATION_TYPE ):
      break;
    case ( CONSTRAINT_UNKNOWN_TYPE ):
    default:
      break;
    }

  } else {
    emit info_update( QString( "[<b>ERROR</b>] cannot edit UNKNOWN constraint type" ) );
  }
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Constraint_Editor& other ) {
    return out;
  }

}
