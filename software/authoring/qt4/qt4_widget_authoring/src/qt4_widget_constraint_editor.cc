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
Qt4_Widget_Constraint_Editor( const Constraint_Task_Space_Region& constraint,
                              Model& robotModel,
                              vector< AffordanceState >& affordanceCollection,
                              const string& urdf_xml_string,
                              unsigned int constraintIndex,
                              QWidget * parent ) : QWidget( parent ),
                                                    _constraint( constraint ),
                                                    _robot_model( robotModel ),
                                                    _robot_affordances(),
                                                    _object_affordances( affordanceCollection ),
                                                    _constraint_index( constraintIndex ),
                                                    _label_id( new QLabel( QString( "C%1" ).arg( _constraint_index ), this ) ),
                                                    _check_box_active( new QCheckBox( this ) ),
                                                    _check_box_visible( new QCheckBox( this ) ),
                                                    _push_button_edit( new QPushButton( QString( "edit" ), this ) ),
                                                    _double_spin_box_time_start( new QDoubleSpinBox( this ) ),
                                                    _double_spin_box_time_end( new QDoubleSpinBox( this ) ),
                                                    _line_edit_metadata( new QLineEdit( QString::fromStdString( _constraint.metadata() ), this ) ),
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

  _label_id->setFixedWidth( 35 );
  _label_id->setAlignment( Qt::AlignRight | Qt::AlignVCenter );
  _check_box_active->setFixedWidth( 25 );
  _check_box_active->setEnabled( true );
  _check_box_visible->setFixedWidth( 25 );
  _check_box_visible->setEnabled( true );
  _push_button_edit->setFixedWidth( 50 );
  _double_spin_box_time_start->setFixedWidth( 70 );
  _double_spin_box_time_start->setRange( 0.1, 1000000.0 );
  _double_spin_box_time_start->setSingleStep( 0.1 );
  _double_spin_box_time_start->setSuffix( " s" );
  _double_spin_box_time_end->setFixedWidth( 70 );
  _double_spin_box_time_end->setRange( 0.1, 1000000.0 );
  _double_spin_box_time_end->setSingleStep( 0.1 );
  _double_spin_box_time_end->setSuffix( " s" );
  _line_edit_metadata->setFixedWidth( 200 );
  _line_edit_description->setFixedWidth( 1024 );

  _double_spin_box_time_start->setValue( _constraint.start() );
  _double_spin_box_time_end->setValue( _constraint.end() );

  QHBoxLayout * widget_layout = new QHBoxLayout();
  widget_layout->addWidget( _label_id );
  widget_layout->addWidget( _check_box_active );
  widget_layout->addWidget( _check_box_visible );
  widget_layout->addWidget( _push_button_edit );
  widget_layout->addWidget( _double_spin_box_time_start );
  widget_layout->addWidget( _double_spin_box_time_end );
  widget_layout->addWidget( _line_edit_metadata );
  widget_layout->addWidget( _line_edit_description );
  setLayout( widget_layout );

  update_constraint( _constraint );

  connect( _check_box_active, SIGNAL( stateChanged( int ) ), this, SLOT( _check_box_active_changed( int ) ) );
  connect( _check_box_visible, SIGNAL( stateChanged( int ) ), this, SLOT( _check_box_visible_changed( int ) ) );
  connect( _push_button_edit, SIGNAL( clicked() ), this, SLOT( _push_button_edit_pressed() ) );
  connect( _double_spin_box_time_start, SIGNAL( valueChanged( double ) ), this, SLOT( _double_spin_box_time_start_value_changed( double ) ) );
  connect( _double_spin_box_time_end, SIGNAL( valueChanged( double ) ), this, SLOT( _double_spin_box_time_end_value_changed( double ) ) );
  connect( _line_edit_metadata, SIGNAL( textEdited( const QString& ) ), this, SLOT( _line_edit_metadata_text_changed( const QString& ) ) );
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
description_from_constraint( const Constraint_Task_Space_Region& constraint ){
  return QString::fromStdString( constraint.description() );
} 

void
Qt4_Widget_Constraint_Editor::
update_constraint( const Constraint_Task_Space_Region& constraint ){
  _constraint = constraint;
  _check_box_active->setCheckState( ( _constraint.active() ? Qt::Checked : Qt::Unchecked ) );
  _check_box_visible->setEnabled( _constraint.active() );
  _check_box_visible->setCheckState( ( _constraint.visible() ? Qt::Checked : Qt::Unchecked ) );
  _push_button_edit->setEnabled( _constraint.active() );
  _double_spin_box_time_start->setEnabled( _constraint.active() );
  _double_spin_box_time_start->setValue( _constraint.start() );
  _double_spin_box_time_end->setEnabled( _constraint.active() );
  _double_spin_box_time_end->setValue( _constraint.end() );
  _line_edit_metadata->setEnabled( _constraint.active() );
  _line_edit_metadata->clear();
  _line_edit_metadata->setText( QString::fromStdString( _constraint.metadata() ) );
  _line_edit_description->setEnabled( _constraint.active() );
  _line_edit_description->clear();
  _line_edit_description->setText( QString::fromStdString( _constraint.description() ) );
  emit constraint_update( _constraint, _constraint_index );
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
highlight_constraint( const QString& id ){
  emit constraint_highlight( id );
  return;
}

void
Qt4_Widget_Constraint_Editor::
_double_spin_box_time_start_value_changed( double start ){
  _constraint.start() = _double_spin_box_time_start->value();
  check_valid_times();
  update_description( QString::fromStdString( _constraint.description() ) );
  emit constraint_update( _constraint );
  emit constraint_update( _constraint, _constraint_index );
  return;
}

void
Qt4_Widget_Constraint_Editor::
_double_spin_box_time_end_value_changed( double end ){
  _constraint.end() = _double_spin_box_time_end->value();
  check_valid_times();
  update_description( QString::fromStdString( _constraint.description() ) );
  emit constraint_update( _constraint );
  emit constraint_update( _constraint, _constraint_index );
  return;
}

void
Qt4_Widget_Constraint_Editor::
_line_edit_metadata_text_changed( const QString& text ){
  _constraint.metadata() = _line_edit_metadata->text().toStdString();
  emit constraint_update( _constraint );
  emit constraint_update( _constraint, _constraint_index );
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
    if( _constraint_editor_popup != NULL ){
      _constraint_editor_popup->close(); 
      delete _constraint_editor_popup;
      _constraint_editor_popup = NULL;
    }
    _constraint.active() = false;
    emit info_update( QString( "[<b>OK</b>] deactivating constraint %1" ).arg( QString::fromStdString( _constraint.id() ) ) );
    break;
  case ( Qt::Checked ):
    _constraint.active() = true;
    emit info_update( QString( "[<b>OK</b>] activating constraint %1" ).arg( QString::fromStdString( _constraint.id() ) ) );
    break;
  default:
    break;
  }
  update_constraint( _constraint );
  return;
}

void
Qt4_Widget_Constraint_Editor::
_check_box_visible_changed( int state ){
  _constraint.visible() = ( _check_box_visible->checkState() == Qt::Checked );
  emit constraint_update( _constraint );
  emit constraint_update( _constraint, _constraint_index );
  return;
}

void
Qt4_Widget_Constraint_Editor::
_push_button_edit_pressed( void ){
  switch( _constraint.type() ){
  case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
    _constraint_editor_popup = new Qt4_Widget_Constraint_Task_Space_Region_Editor( _constraint, _robot_model, _object_affordances, this );
    connect( this, SIGNAL( constraint_update( const Constraint_Task_Space_Region& ) ), _constraint_editor_popup, SLOT( update_constraint( const Constraint_Task_Space_Region& ) ) );
    connect( _constraint_editor_popup, SIGNAL( constraint_update( const Constraint_Task_Space_Region& ) ), this, SLOT( update_constraint( const Constraint_Task_Space_Region& ) ) );
    connect( _constraint_editor_popup, SIGNAL( constraint_highlight( const QString& ) ), this, SLOT( highlight_constraint( const QString& ) ) );
    _constraint_editor_popup->show();
    emit info_update( QString( "[<b>OK</b>] launching editor for constraint %1" ).arg( QString::fromStdString( _constraint.id() ) ) );
    break;
  case ( CONSTRAINT_CONFIGURATION_TYPE ):
  case ( CONSTRAINT_UNKNOWN_TYPE ):
  default:
    break;
  }
  update_description( QString::fromStdString( _constraint.description() ) );
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Qt4_Widget_Constraint_Editor& other ) {
    return out;
  }

}
