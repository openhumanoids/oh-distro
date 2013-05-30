#include <QtGui/QHBoxLayout>

#include "authoring/qt4_widget_constraint_task_space_region_editor.h"
#include "authoring/constraint_configuration.h" // TODO move to editor when defined for configuration space
#include "qt4/qt4_widget_gfe_object.h" // TODO move to editor when defined for configuration space
#include "authoring/qt4_widget_constraint_editor.h"
#include <qt4/qt4_widget_gfe_control.h>

using namespace std;
using namespace boost;
using namespace urdf;
using namespace affordance;
using namespace authoring;

Qt4_Widget_Constraint_Editor::
Qt4_Widget_Constraint_Editor( Constraint *& constraint,
                              Model& robotModel,
                              vector< AffordanceState >& affordanceCollection,
                              const string& urdf_xml_string,
                              const string& id,
                              QWidget * parent ) : QWidget( parent ),
                                                    _constraint( constraint ),
                                                    _robot_model( robotModel ),
                                                    _robot_affordances(),
                                                    _object_affordances( affordanceCollection ),
                                                    _id( id ),
                                                    _label_id( new QLabel( QString::fromStdString( id ), this ) ),
                                                    _check_box_active( new QCheckBox( this ) ),
                                                    _combo_box_type( new QComboBox( this ) ),
                                                    _push_button_edit( new QPushButton( QString( "edit" ), this ) ),
                                                    _double_spin_box_time_start( new QDoubleSpinBox( this ) ),
                                                    _double_spin_box_time_end( new QDoubleSpinBox( this ) ),
                                                    _line_edit_description( new QLineEdit( description_from_constraint( _constraint ), this ) ),
                                                    _constraint_editor_popup( NULL ) {

  _double_spin_box_time_start->setToolTip("the absolute start time for this constraint, in seconds");
  _double_spin_box_time_end->setToolTip("the absolute end time for this constraint, in seconds");

  _line_edit_description->setReadOnly(true);

  _urdf_xml_string = urdf_xml_string;
  vector< shared_ptr< Link > > links;
  _robot_model.getLinks( links );
  for( vector< shared_ptr< Link > >::iterator it1 = links.begin(); it1 != links.end(); it1++ ){
    for( map< string, shared_ptr< vector< shared_ptr< Collision > > > >::iterator it2 = (*it1)->collision_groups.begin(); it2 != (*it1)->collision_groups.end(); it2++ ){
      _robot_affordances.push_back( pair< string, string >( (*it1)->name, it2->first ) );
    }
  }

  for( unsigned int i = 0; i < NUM_CONSTRAINT_TYPES; i++ ){
    _combo_box_type->addItem( QString::fromStdString( Constraint::constraint_type_t_to_std_string( ( constraint_type_t )( i ) ) ) );
  }

  _label_id->setFixedWidth( 35 );
  _check_box_active->setFixedWidth( 25 );
  _combo_box_type->setFixedWidth( 65 );
  _push_button_edit->setFixedWidth( 50 );
  _double_spin_box_time_start->setFixedWidth( 70 );
  _double_spin_box_time_start->setRange( 0.1, 1000000.0 );
  _double_spin_box_time_start->setSingleStep( 0.1 );
  _double_spin_box_time_start->setSuffix( " s" );
  _double_spin_box_time_end->setFixedWidth( 70 );
  _double_spin_box_time_end->setRange( 0.1, 1000000.0 );
  _double_spin_box_time_end->setSingleStep( 0.1 );
  _double_spin_box_time_end->setSuffix( " s" );
  _line_edit_description->setFixedWidth( 1024 );

  if(_constraint){
    _double_spin_box_time_start->setValue( _constraint->start() );
    _double_spin_box_time_end->setValue( _constraint->end() );
  }

  QHBoxLayout * widget_layout = new QHBoxLayout();
  widget_layout->addWidget( _check_box_active );
  widget_layout->addWidget( _label_id );
  widget_layout->addWidget( _combo_box_type );
  widget_layout->addWidget( _push_button_edit );
  widget_layout->addWidget( _double_spin_box_time_start );
  widget_layout->addWidget( _double_spin_box_time_end );
  widget_layout->addWidget( _line_edit_description );
  setLayout( widget_layout );

  connect( _check_box_active, SIGNAL( stateChanged( int ) ), this, SLOT( _check_box_active_changed( int ) ) );
  connect( _combo_box_type, SIGNAL( currentIndexChanged( int ) ), this, SLOT( _combo_box_type_changed( int ) ) );
  connect( _push_button_edit, SIGNAL( clicked() ), this, SLOT( _push_button_edit_pressed() ) );
  connect( _double_spin_box_time_start, SIGNAL( valueChanged( double ) ), this, SLOT( _double_spin_box_time_start_value_changed( double ) ) );
  connect( _double_spin_box_time_end, SIGNAL( valueChanged( double ) ), this, SLOT( _double_spin_box_time_end_value_changed( double ) ) );

  if( constraint != NULL ){
    _combo_box_type->setEnabled( constraint->active() );
    _push_button_edit->setEnabled( constraint->active() );
    _double_spin_box_time_start->setEnabled( constraint->active() );
    _double_spin_box_time_end->setEnabled( constraint->active() );
  } else {
    _combo_box_type->setEnabled( false );
    _push_button_edit->setEnabled( false );
    _double_spin_box_time_start->setEnabled( false );
    _double_spin_box_time_end->setEnabled( false );
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

QString
Qt4_Widget_Constraint_Editor::
description_from_constraint( const Constraint* constraint ){
  if( constraint != NULL ){
    return QString::fromStdString( constraint->description() );
  } else {
    return QString( "N/A" );
  }
} 

void
Qt4_Widget_Constraint_Editor::
update_constraint( void ){
  if( _constraint != NULL ){
    if( _constraint->active() ){
      _check_box_active->setCheckState( Qt::Checked );
    } else {
      _check_box_active->setCheckState( Qt::Unchecked );
    }
    _combo_box_type->setCurrentIndex( _constraint->type() );
    _double_spin_box_time_start->setValue( _constraint->start() );
    _double_spin_box_time_end->setValue( _constraint->end() );
  } else {
    _check_box_active->setCheckState( Qt::Unchecked );
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
update_description( const QString& description ){
  _line_edit_description->clear();
  _line_edit_description->setText( description );
  update();
  return;
}

void
Qt4_Widget_Constraint_Editor::
_double_spin_box_time_start_value_changed( double start ){
  if( _constraint != NULL ){
    _constraint->start() = _double_spin_box_time_start->value();
    cout <<"_constraint->start() = " <<  _constraint->start() << endl;
    check_valid_times();
    update_description( QString::fromStdString( _constraint->description() ) );
  } else {
    update_description( "N/A" );
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
_double_spin_box_time_end_value_changed( double end ){
  if( _constraint != NULL ){
    _constraint->end() = _double_spin_box_time_end->value();
    check_valid_times();
    update_description( QString::fromStdString( _constraint->description() ) );
  } else {
    update_description( "N/A" );
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
check_valid_times() {
    QString invalid = "QDoubleSpinBox { background-color: #ff0000; color: white }";
    QString valid = "";
    _double_spin_box_time_start->setStyleSheet((_double_spin_box_time_start->value() > _double_spin_box_time_end->value()) ? invalid : valid);
    _double_spin_box_time_end->setStyleSheet((_double_spin_box_time_start->value() > _double_spin_box_time_end->value()) ? invalid : valid);
}

void
Qt4_Widget_Constraint_Editor::
_check_box_active_changed( int state ){
  switch( _check_box_active->checkState() ){
  case ( Qt::Unchecked ):
  case ( Qt::PartiallyChecked ):
    _combo_box_type->setEnabled( false );
    _push_button_edit->setEnabled( false );
    _double_spin_box_time_start->setEnabled( false );
    _double_spin_box_time_end->setEnabled( false );
    if( _constraint_editor_popup != NULL ){
      _constraint_editor_popup->close(); 
      delete _constraint_editor_popup;
      _constraint_editor_popup = NULL;
    }
    if( _constraint != NULL ){
      emit info_update( QString( "[<b>OK</b>] deactivating constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      _constraint->active() = false;
    } else {
      emit info_update( QString( "[<b>OK</b>] deactivating constraint %1" ).arg( QString::fromStdString( _id ) ) );
    }
    break;
  case ( Qt::Checked ):
    _combo_box_type->setEnabled( true );
    if( _constraint != NULL ){
      _constraint->active() = true;
      if( _constraint->type() == _combo_box_type->currentIndex() ){
        _push_button_edit->setEnabled( true );
        _double_spin_box_time_start->setEnabled( true );
        _double_spin_box_time_end->setEnabled( true );
      }
      emit info_update( QString( "[<b>OK</b>] activating constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
    } else {
      emit info_update( QString( "[<b>OK</b>] activating constraint %1" ).arg( QString::fromStdString( _id ) ) );
    }
    break;
  default:
    break;
  }
  if( _constraint != NULL ){
    update_description( QString::fromStdString( _constraint->description() ) );
    _double_spin_box_time_start->setValue( _constraint->start() );
    _double_spin_box_time_end->setValue( _constraint->end() );
  } else {
    update_description( "N/A" );
  }
  return;
}

void
Qt4_Widget_Constraint_Editor::
_combo_box_type_changed( int index ){
  if( _constraint != NULL ){
    if( _constraint->type() != _combo_box_type->currentIndex() ){
      delete _constraint;
      _constraint = NULL;
      emit info_update( QString( "[<b>OK</b>] deleted constraint %1" ).arg( QString::fromStdString( _id ) ) );
      switch( _combo_box_type->currentIndex() ){
      case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
        _constraint = new Constraint_Task_Space_Region( _id );
        emit info_update( QString( "[<b>OK</b>] instatiated new task space region constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
        break;
      case ( CONSTRAINT_CONFIGURATION_TYPE ):
        _constraint = new Constraint_Configuration( _id );
        emit info_update( QString( "[<b>OK</b>] instatiated new configuration constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
        break;
      case ( CONSTRAINT_UNKNOWN_TYPE ):
      default:
        break;
      }
    }    
  } else {
    switch( _combo_box_type->currentIndex() ){
    case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
      _constraint = new Constraint_Task_Space_Region( _id );
      emit info_update( QString( "[<b>OK</b>] instatiated new task space region constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_CONFIGURATION_TYPE ):
      _constraint = new Constraint_Configuration( _id );
      emit info_update( QString( "[<b>OK</b>] instatiated new configuration constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_UNKNOWN_TYPE ):
    default:
      break;
    }
  }

  switch( _combo_box_type->currentIndex() ){
  case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
    _push_button_edit->setEnabled( true );
    _double_spin_box_time_start->setEnabled( true );
    _double_spin_box_time_end->setEnabled( true );
    break;
  case ( CONSTRAINT_CONFIGURATION_TYPE ):
    _push_button_edit->setEnabled( true );
    _double_spin_box_time_start->setEnabled( true );
    _double_spin_box_time_end->setEnabled( true );
    break;
  case ( CONSTRAINT_UNKNOWN_TYPE ):
  default:
    _push_button_edit->setEnabled( false );
    _double_spin_box_time_start->setEnabled( false );
    _double_spin_box_time_end->setEnabled( false );
    break;
  }
  if( _constraint != NULL ){
    update_description( QString::fromStdString( _constraint->description() ) );
    _double_spin_box_time_start->setValue( _constraint->start() );
    _double_spin_box_time_end->setValue( _constraint->end() );
  } else {
    update_description( "N/A" );
  }
  return;
}
void
Qt4_Widget_Constraint_Editor::
show_visualizer( void ){
    emit constraint_selected( QString::fromStdString( _constraint->id() ) );
}

void
Qt4_Widget_Constraint_Editor::
_push_button_edit_pressed( void ){
  if( _constraint != NULL ){
    switch( _constraint->type() ){
    case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
      _constraint_editor_popup = new Qt4_Widget_Constraint_Task_Space_Region_Editor( dynamic_cast< Constraint_Task_Space_Region* >( _constraint ), _robot_model, _object_affordances, this );
      connect( _constraint_editor_popup, SIGNAL( description_update( const QString& ) ), this, SLOT( update_description( const QString& ) ) );
      connect( _constraint_editor_popup, SIGNAL( widget_selected ( void ) ), this, SLOT( show_visualizer( void ) ) );
      connect( _constraint_editor_popup, SIGNAL( highlight_parent_link_by_name ( const QString& ) ), this, SIGNAL( highlight_link_by_name( const QString& ) ) );
      _constraint_editor_popup->show();
      emit info_update( QString( "[<b>OK</b>] launching editor for constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_CONFIGURATION_TYPE ):
      _constraint_visualizer_popup = new qt4::Qt4_Widget_GFE_Object( _urdf_xml_string );
      _constraint_visualizer_popup->show();
      _constraint_editor_popup = new qt4::Qt4_Widget_GFE_Control();
      _constraint_editor_popup->show();
      QObject::connect( _constraint_editor_popup, SIGNAL( state_update( state::State_GFE& ) ), _constraint_visualizer_popup, SLOT( update_state( state::State_GFE& ) ) );
      emit info_update( QString( "[<b>OK</b>] launching editor for constraint %1" ).arg( QString::fromStdString( _constraint->id() ) ) );
      break;
    case ( CONSTRAINT_UNKNOWN_TYPE ):
    default:
      break;
    }
  } else {
    emit info_update( QString( "[<b>ERROR</b>] cannot edit UNKNOWN constraint type" ) );
  }
  if( _constraint != NULL ){
    update_description( QString::fromStdString( _constraint->description() ) );
  } else {
    update_description( "N/A" );
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
